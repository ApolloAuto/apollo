#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import math
from reference_path import ReferencePath
from local_path import LocalPath
from numpy.polynomial.polynomial import polyval


class PathDecider:
    def __init__(self, enable_routing_aid, enable_nudge, enable_change_lane):
        self.MINIMUM_PATH_LENGTH = 5
        self.MAX_LAT_CHANGE = 0.1
        self.last_init_lat = None
        self.ref = ReferencePath()
        self.enable_routing_aid = enable_routing_aid
        self.enable_nudge = enable_nudge
        self.enable_change_lane = enable_change_lane
        self.path_range = 10

    def get_path_by_lm(self, mobileye, adv):
        return self.ref.get_ref_path_by_lm(mobileye, adv)

    def get_path_by_lmr(self, perception, routing, adv):
        path_x, path_y, path_len = self.ref.get_ref_path_by_lmr(perception,
                                                                routing,
                                                                adv)
        if self.enable_nudge:
            path_x, path_y, path_len = self.nudge_process(path_x, path_y,
                                                          path_len)
        return path_x, path_y, path_len

    def nudge_process(self, final_path, obstacle_decider):
        obstacle_decider.process_path_obstacle(final_path)
        left_dist = 999
        right_dist = 999
        for obs_id, lat_dist in obstacle_decider.obstacle_lat_dist.items():
            if lat_dist < 0:
                left_dist = lat_dist
            else:
                right_dist = lat_dist
        print(left_dist, right_dist)
        return final_path

    def get(self, perception, routing, adv):
        if self.enable_routing_aid:
            return self.get_path_by_lmr(perception, routing, adv)
        else:
            return self.get_path_by_lm(perception, adv)

    def get_path(self, perception, routing, adv, obstacle_decider):
        self.path_range = self._get_path_range(adv.speed_mps)
        if self.enable_routing_aid and adv.is_ready():
            return self.get_routing_path(perception, routing, adv,
                                         obstacle_decider)
        else:
            return self.get_lane_marker_path(perception)

    def get_routing_path(self, perception, routing, adv, obstacle_decider):

        routing_path = routing.get_local_path(adv, self.path_range + 1)
        perception_path = perception.get_lane_marker_middle_path(
            self.path_range + 1)

        quality = perception.right_lm_quality + perception.left_lm_quality
        quality = quality / 2.0

        if routing_path.range() >= self.path_range \
                and routing.human \
                and routing_path.init_y() <= 3:
            # "routing only"
            init_y_routing = routing_path.init_y()
            init_y = self._smooth_init_y(init_y_routing)
            routing_path.shift(init_y - routing_path.init_y())
            if self.enable_nudge:
                obstacle_decider.process_path_obstacle(routing_path)
                left_nudgable, right_nudgable = \
                    obstacle_decider.get_adv_left_right_nudgable_dist(
                        routing_path)
                nudge_dist = obstacle_decider.get_nudge_distance(left_nudgable,
                                                                 right_nudgable)
                smoothed_nudge_dist = self._smooth_init_y(nudge_dist)
                if smoothed_nudge_dist != 0:
                    print(smoothed_nudge_dist)
                routing_path.shift(smoothed_nudge_dist)
            return routing_path

        init_y = self._smooth_init_y(perception_path.init_y())
        if routing_path.range() < self.path_range:
            # "perception only"
            perception_path.shift(init_y - perception_path.init_y())
            return perception_path

        # "hybrid"
        init_y = perception_path.init_y()
        routing_path.shift(init_y - routing_path.init_y())
        perception_path.shift(init_y - routing_path.init_y())
        routing_path.merge(perception_path, quality)

        return routing_path

    def get_lane_marker_path(self, perception):
        path = perception.get_lane_marker_middle_path(perception,
                                                      self.path_range)
        init_y = path.init_y()
        smoothed_init_y = self._smooth_init_y(init_y)
        path.shift(smoothed_init_y - init_y)
        return path

    def _get_path_range(self, speed_mps):
        path_length = self.MINIMUM_PATH_LENGTH
        current_speed = speed_mps
        if current_speed is not None:
            if path_length < current_speed * 2:
                path_length = math.ceil(current_speed * 2)
        return int(path_length)

    def _smooth_init_y(self, init_y):
        if init_y > 0.2:
            init_y = 0.2
        if init_y < -0.2:
            init_y = -0.2
        return init_y


if __name__ == "__main__":
    import rospy
    from std_msgs.msg import String
    import matplotlib.pyplot as plt
    from modules.localization.proto import localization_pb2
    from modules.canbus.proto import chassis_pb2
    from ad_vehicle import ADVehicle
    import matplotlib.animation as animation
    from modules.drivers.proto import mobileye_pb2
    from provider_routing import RoutingProvider
    from provider_mobileye import MobileyeProvider

    def localization_callback(localization_pb):
        ad_vehicle.update_localization(localization_pb)

    def routing_callback(routing_str):
        routing.update(routing_str)

    def chassis_callback(chassis_pb):
        ad_vehicle.update_chassis(chassis_pb)

    def mobileye_callback(mobileye_pb):
        mobileye.update(mobileye_pb)
        mobileye.process_lane_markers()

    def update(frame):
        if not ad_vehicle.is_ready():
            return
        left_path = mobileye.get_left_lane_marker_path()
        left_x, left_y = left_path.get_xy()
        left_lm.set_xdata(left_x)
        left_lm.set_ydata(left_y)

        right_path = mobileye.get_right_lane_marker_path()
        right_x, right_y = right_path.get_xy()
        right_lm.set_xdata(right_x)
        right_lm.set_ydata(right_y)

        middle_path = mobileye.get_lane_marker_middle_path(128)
        middle_x, middle_y = middle_path.get_xy()
        middle_lm.set_xdata(middle_x)
        middle_lm.set_ydata(middle_y)

        fpath = path_decider.get_path(mobileye, routing, ad_vehicle)
        fpath_x, fpath_y = fpath.get_xy()
        final_path.set_xdata(fpath_x)
        final_path.set_ydata(fpath_y)
        # ax.autoscale_view()
        # ax.relim()

    ad_vehicle = ADVehicle()
    routing = RoutingProvider()
    mobileye = MobileyeProvider()
    path_decider = PathDecider(True, False, False)

    rospy.init_node("path_decider_debug", anonymous=True)
    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     localization_callback)
    rospy.Subscriber('/apollo/navigation/routing',
                     String, routing_callback)
    rospy.Subscriber('/apollo/canbus/chassis',
                     chassis_pb2.Chassis,
                     chassis_callback)
    rospy.Subscriber('/apollo/sensor/mobileye',
                     mobileye_pb2.Mobileye,
                     mobileye_callback)

    fig = plt.figure()
    ax = plt.subplot2grid((1, 1), (0, 0), rowspan=1, colspan=1)
    left_lm, = ax.plot([], [], 'b-')
    right_lm, = ax.plot([], [], 'b-')
    middle_lm, = ax.plot([], [], 'k-')
    final_path, = ax.plot([], [], 'r-')

    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.set_xlim([-2, 128])
    ax.set_ylim([-5, 5])
    # ax2.axis('equal')
    plt.show()
