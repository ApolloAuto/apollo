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

from shapely.geometry import LineString
from shapely.geometry import Point


class ObstacleDecider:
    def __init__(self):
        self.obstacle_lat_ttc = {}
        self.obstacle_lon_ttc = {}
        self.obstacle_lat_dist = {}
        self.obstacle_lon_dist = {}
        self.front_edge_to_center = 3.89
        self.back_edge_to_center = 1.043
        self.left_edge_to_center = 1.055
        self.right_edge_to_center = 1.055
        self.LAT_DIST = 0.9
        self.mobileye = None
        self.path_obstacle_processed = False
        self.default_lane_width = 3.3

    def update(self, mobileye):
        self.mobileye = mobileye
        self.path_obstacle_processed = False

    def process_path_obstacle(self, fpath):
        if self.path_obstacle_processed:
            return

        path_x, path_y = fpath.get_xy()
        self.obstacle_lat_dist = {}

        path = []
        self.mobileye.process_obstacles()
        for i in range(len(path_x)):
            path.append((path_x[i], path_y[i]))
        line = LineString(path)

        for obs_id, obstacle in self.mobileye.obstacles.items():
            point = Point(obstacle.x, obstacle.y)
            dist = line.distance(point)
            if dist < self.LAT_DIST + obstacle.width + self.left_edge_to_center:
                proj_len = line.project(point)
                if proj_len == 0 or proj_len >= line.length:
                    continue
                p1 = line.interpolate(proj_len)
                if (proj_len + 1) > line.length:
                    p2 = line.interpolate(line.length)
                else:
                    p2 = line.interpolate(proj_len + 1)
                d = (point.x - p1.x) * (p2.y - p1.y) - (point.y - p1.y) * (
                    p2.x - p1.x)
                if d > 0:
                    dist *= -1
                self.obstacle_lat_dist[obstacle.obstacle_id] = dist

        self.path_obstacle_processed = True
        # print self.obstacle_lat_dist

    def get_adv_left_right_nudgable_dist(self, fpath):
        left_nudgable = 0
        right_nudgable = 0
        routing_y = fpath.init_y()
        if routing_y <= 0:
            left_nudgable = self.default_lane_width / 2.0 \
                - abs(routing_y) \
                - self.left_edge_to_center
            right_nudgable = self.default_lane_width / 2.0 \
                + abs(routing_y) \
                - self.right_edge_to_center
        else:
            left_nudgable = self.default_lane_width / 2.0 \
                + abs(routing_y) \
                - self.left_edge_to_center
            right_nudgable = self.default_lane_width / 2.0 \
                - abs(routing_y) \
                - self.right_edge_to_center

        return left_nudgable, -1 * right_nudgable

    def get_nudge_distance(self, left_nudgable, right_nudgable):
        left_nudge = None
        right_nudge = None
        for obs_id, lat_dist in self.obstacle_lat_dist.items():
            if lat_dist >= 0:
                actual_dist = abs(lat_dist) \
                    - self.mobileye.obstacles[obs_id].width / 2.0 \
                    - self.left_edge_to_center
                if self.LAT_DIST > actual_dist > 0.2:
                    if right_nudge is None:
                        right_nudge = -1 * (self.LAT_DIST - actual_dist)
                    elif right_nudge > -1 * (self.LAT_DIST - actual_dist):
                        right_nudge = -1 * (self.LAT_DIST - actual_dist)
            else:
                actual_dist = abs(lat_dist) \
                    - self.mobileye.obstacles[obs_id].width / 2.0 \
                    - self.left_edge_to_center
                if self.LAT_DIST > actual_dist > 0.2:
                    if left_nudge is None:
                        left_nudge = self.LAT_DIST - actual_dist
                    elif left_nudge < self.LAT_DIST - actual_dist:
                        left_nudge = self.LAT_DIST - actual_dist
        if left_nudge is None and right_nudge is None:
            return 0
        if left_nudge is not None and right_nudge is not None:
            return 0
        if left_nudge is not None:
            if left_nudgable < left_nudge:
                return left_nudgable
            else:
                return left_nudge
        if right_nudge is not None:
            if abs(right_nudgable) > abs(right_nudge):
                return right_nudgable
            else:
                return right_nudge


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
    from path_decider import PathDecider

    def localization_callback(localization_pb):
        ad_vehicle.update_localization(localization_pb)

    def routing_callback(routing_str):
        routing.update(routing_str)

    def chassis_callback(chassis_pb):
        ad_vehicle.update_chassis(chassis_pb)

    def mobileye_callback(mobileye_pb):
        global fpath
        mobileye.update(mobileye_pb)
        mobileye.process_lane_markers()
        fpath = path_decider.get_path(mobileye, routing, ad_vehicle,
                                      obs_decider)
        obs_decider.update(mobileye)
        obs_decider.process_path_obstacle(fpath)
        print("nudge distance = ", obs_decider.get_nudge_distance())

    def update(frame):
        if not ad_vehicle.is_ready():
            return
        x = []
        y = []
        for obs_id, obs in mobileye.obstacles.items():
            x.append(obs.x)
            y.append(obs.y)
        obstacles_points.set_xdata(x)
        obstacles_points.set_ydata(y)

        if fpath is not None:
            px, py = fpath.get_xy()
            path_line.set_xdata(px)
            path_line.set_ydata(py)

    fpath = None
    ad_vehicle = ADVehicle()
    routing = RoutingProvider()
    mobileye = MobileyeProvider()
    obs_decider = ObstacleDecider()
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
    obstacles_points, = ax.plot([], [], 'ro')
    path_line, = ax.plot([], [], 'b-')

    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.set_xlim([-2, 128])
    ax.set_ylim([-5, 5])
    # ax2.axis('equal')
    plt.show()
