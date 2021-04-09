#!/usr/bin/env python

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
import shapely
from shapely.geometry import Point
from shapely.geometry import LineString
from lanemarker_corrector import LaneMarkerCorrector
from numpy.polynomial.polynomial import polyval
from local_path import LocalPath


class Obstacle:
    def __init__(self, obstacle_id, x, y):
        self.obstacle_id = obstacle_id
        self.x = x
        self.y = y
        self.lane = 0
        self.length = -1
        self.width = -1
        self.rel_speed = -1


def cut(line, distance):
    # Cuts a line in two at a distance from its starting point
    # This is taken from shapely manual
    if distance <= 0.0 or distance >= line.length:
        return [LineString(line)]
    coords = list(line.coords)
    for i, p in enumerate(coords):
        pd = line.project(Point(p))
        if pd == distance:
            return [
                LineString(coords[:i + 1]),
                LineString(coords[i:])]
        if pd > distance:
            cp = line.interpolate(distance)
            return [
                LineString(coords[:i] + [(cp.x, cp.y)]),
                LineString([(cp.x, cp.y)] + coords[i:])]


class MobileyeProvider:
    def __init__(self):
        self.mobileye_pb = None
        self.right_lm_coef = [1, 0, 0, 0]
        self.left_lm_coef = [1, 0, 0, 0]
        self.right_lane_marker_range = 0
        self.left_lane_marker_range = 0
        self.obstacles = {}
        self.right_lm_quality = 0.0
        self.left_lm_quality = 0.0

        self.history_left_lines = []
        self.last_speed = None
        self.last_heading = None

        self.lane_marker_processed = False
        self.history_processed = False
        self.obstacles_processed = False

    def update(self, mobileye_pb):
        self.lane_marker_processed = False
        self.history_processed = False
        self.obstacles_processed = False

        self.mobileye_pb = mobileye_pb
        self.process_lane_markers()

    def process_lane_markers(self):
        if self.lane_marker_processed:
            return
        rc0 = self.mobileye_pb.lka_768.position
        rc1 = self.mobileye_pb.lka_769.heading_angle
        rc2 = self.mobileye_pb.lka_768.curvature
        rc3 = self.mobileye_pb.lka_768.curvature_derivative
        self.right_lane_marker_range = self.mobileye_pb.lka_769.view_range
        self.right_lm_coef = [rc0, rc1, rc2, rc3]

        lc0 = self.mobileye_pb.lka_766.position
        lc1 = self.mobileye_pb.lka_767.heading_angle
        lc2 = self.mobileye_pb.lka_766.curvature
        lc3 = self.mobileye_pb.lka_766.curvature_derivative
        self.left_lane_marker_range = self.mobileye_pb.lka_767.view_range
        self.left_lm_coef = [lc0, lc1, lc2, lc3]

        self.left_lm_quality = self.mobileye_pb.lka_766.quality / 3.0
        self.right_lm_quality = self.mobileye_pb.lka_768.quality / 3.0

        self.lane_marker_processed = True

    def process_history(self, heading, speed):
        if self.history_processed:
            return
        history_len = len(self.history_left_lines)
        if self.last_speed is None:
            self.history_left_lines = []
        if history_len > 30:
            self.history_left_lines = self.history_left_lines[history_len - 30:]
        for i in range(len(self.history_left_lines)):
            line = self.history_left_lines[i]
            if line is None:
                continue

            x_dist = (self.last_speed + speed) / 2.0 * 0.1
            distance = x_dist  # line.project(Point(x_dist, 0.0))
            if distance > line.length:
                self.history_left_lines[i] = None
                continue
            elif distance <= 0:
                self.history_left_lines[i] = line
                continue
            else:
                lines = cut(line, distance)
                if len(lines) >= 2:
                    line = lines[1]
                    shift = line.coords[0][0]
                    newc = []
                    for k in range(len(line.coords)):
                        newc.append(
                            (line.coords[k][0] - shift, line.coords[k][1]))
                    line.coords = newc

                    line = shapely.affinity.rotate(line,
                                                   heading - self.last_heading,
                                                   origin=line.coords[0],
                                                   use_radians=True)
                    self.history_left_lines[i] = line
                else:
                    self.history_left_lines[i] = None

        points = []
        for x in range(int(self.left_lane_marker_range)):
            y = polyval(x, self.left_lm_coef)
            points.append((x, y))
        line = LineString(points)
        if line.length > 0.1:
            self.history_left_lines.append(line)
        else:
            self.history_left_lines.append(None)

        self.last_speed = speed
        self.last_heading = heading

        self.history_processed = True

    def process_obstacles(self):
        if self.obstacles_processed:
            return

        if self.mobileye_pb is None:
            return
        self.obstacles = {}
        for i in range(len(self.mobileye_pb.details_739)):
            obstacle_id = self.mobileye_pb.details_739[i].obstacle_id
            x = self.mobileye_pb.details_739[i].obstacle_pos_x
            y = self.mobileye_pb.details_739[i].obstacle_pos_y
            obstacle = Obstacle(obstacle_id, x, y)
            obstacle.rel_speed = self.mobileye_pb.details_739[
                i].obstacle_rel_vel_x
            if i < len(self.mobileye_pb.details_73a):
                obstacle.lane = self.mobileye_pb.details_73a[i].obstacle_lane
                obstacle.length = self.mobileye_pb.details_73a[
                    i].obstacle_length
                obstacle.width = self.mobileye_pb.details_73a[i].obstacle_width
            self.obstacles[obstacle.obstacle_id] = obstacle

        self.obstacles_processed = True

    def routing_correct(self, routing_provider, localization_provider):
        routing_segment = routing_provider.get_segment()

        vx = localization_provider.localization_pb.pose.position.x
        vy = localization_provider.localization_pb.pose.position.y
        heading = localization_provider.localization_pb.pose.heading
        position = (vx, vy)
        corrector = LaneMarkerCorrector(self.left_lm_coef,
                                        self.right_lm_coef)
        self.left_lm_coef, self.right_lm_coef = \
            corrector.correct(position, heading, routing_segment)

    def get_right_lane_marker_path(self):
        points = []
        for x in range(int(self.right_lane_marker_range)):
            y = -1 * polyval(x, self.right_lm_coef)
            points.append([x, y])
        path = LocalPath(points)
        return path

    def get_left_lane_marker_path(self):
        points = []
        for x in range(int(self.left_lane_marker_range)):
            y = -1 * polyval(x, self.left_lm_coef)
            points.append([x, y])
        path = LocalPath(points)
        return path

    def get_lane_marker_middle_path(self, path_range):
        init_y_perception = (self.right_lm_coef[0] +
                             self.left_lm_coef[0]) / -2.0
        path_coef = [0, 0, 0, 0]

        path_coef[0] = -1 * init_y_perception
        quality = self.right_lm_quality + self.left_lm_quality
        if quality > 0:
            for i in range(1, 4):
                path_coef[i] = (self.right_lm_coef[i] *
                                self.right_lm_quality +
                                self.left_lm_coef[i] *
                                self.left_lm_quality) / quality
        points = []
        for x in range(path_range):
            y = -1 * polyval(x, path_coef)
            points.append([x, y])

        return LocalPath(points)


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


    def localization_callback(localization_pb):
        ad_vehicle.update_localization(localization_pb)


    def routing_callback(routing_str):
        routing.update(routing_str)


    def chassis_callback(chassis_pb):
        ad_vehicle.update_chassis(chassis_pb)


    def mobileye_callback(mobileye_pb):
        mobileye.update(mobileye_pb)
        mobileye.process_lane_markers();


    def update(frame):
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
        # ax.autoscale_view()
        # ax.relim()


    ad_vehicle = ADVehicle()
    routing = RoutingProvider()
    mobileye = MobileyeProvider()

    rospy.init_node("routing_debug", anonymous=True)
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
    right_lm, = ax.plot([], [], 'g-')
    middle_lm, = ax.plot([], [], 'r-')

    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.set_xlim([-2, 128])
    ax.set_ylim([-5, 5])
    # ax2.axis('equal')
    plt.show()
