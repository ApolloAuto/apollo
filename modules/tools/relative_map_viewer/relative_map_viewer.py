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

import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from modules.map.relative_map.proto import navigation_pb2
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2

ax = None
map_msg = None
planning_msg = None
navigation_path_x = []
navigation_path_y = []
localization_pb = None


def evaluate_poly(c0, c1, c2, c3, x):
    return ((c3 * x + c2) * x + c1) * x + c0


def localization_callback(localization_pb2):
    global localization_pb
    localization_pb = localization_pb2


def draw_lane_boundary(lane, ax, color_val, lane_marker):
    """draw boundary"""
    left_c0 = float(lane_marker.left_lane_marker.c0_position)
    left_c1 = float(lane_marker.left_lane_marker.c1_heading_angle)
    left_c2 = float(lane_marker.left_lane_marker.c2_curvature)
    left_c3 = float(lane_marker.left_lane_marker.c3_curvature_derivative)

    for curve in lane.left_boundary.curve.segment:
        if curve.HasField('line_segment'):
            px = []
            py = []

            for p in curve.line_segment.point:
                px.append(float(p.y))
                py.append(float(p.x))

            ax.plot(px, py, ls='-', c=color_val, alpha=0.5)

    # draw lane marker
    px_lane_marker = []
    py_lane_marker = []
    for x in range(int(lane_marker.left_lane_marker.view_range)):
        px_lane_marker.append(evaluate_poly(left_c0, left_c1,
                                             left_c2, left_c3,
                                             float(x)))
        py_lane_marker.append(float(x))
    ax.plot(px_lane_marker, py_lane_marker, ls='--', c='g', alpha=0.5)

    right_c0 = float(lane_marker.right_lane_marker.c0_position)
    right_c1 = float(lane_marker.right_lane_marker.c1_heading_angle)
    right_c2 = float(lane_marker.right_lane_marker.c2_curvature)
    right_c3 = float(lane_marker.right_lane_marker.c3_curvature_derivative)
    for curve in lane.right_boundary.curve.segment:
        if curve.HasField('line_segment'):
            px = []
            py = []
            for p in curve.line_segment.point:
                px.append(float(p.y))
                py.append(float(p.x))
            ax.plot(px, py, ls='-', c=color_val, alpha=0.5)

    # draw lane marker
    px_lane_marker = []
    py_lane_marker = []
    for x in range(int(lane_marker.right_lane_marker.view_range)):
        px_lane_marker.append(evaluate_poly(right_c0, right_c1,
                                             right_c2, right_c3,
                                             float(x)))
        py_lane_marker.append(float(x))
    ax.plot(px_lane_marker, py_lane_marker, ls='--', c='g', alpha=0.5)

    if localization_pb is None:
        return

    vx = localization_pb.pose.position.x
    vy = localization_pb.pose.position.y
    heading = localization_pb.pose.heading

    path_x = [x - vx for x in navigation_path_x]
    path_y = [y - vy for y in navigation_path_y]

    routing_x = []
    routing_y = []

    for i in range(len(path_x)):
        x = path_x[i]
        y = path_y[i]
        newx = x * math.cos(- heading + 1.570796) - y * math.sin(
            -heading + 1.570796)
        newy = y * math.cos(- heading + 1.570796) + x * math.sin(
            -heading + 1.570796)

        routing_x.append(-1 * newx)
        routing_y.append(newy)

    ax.plot(routing_x, routing_y, c='k')


def navigation_callback(navigation_info_pb):
    global navigation_path_x, navigation_path_y
    navigation_path_x = []
    navigation_path_y = []
    for navi_path in navigation_info_pb.navigation_path:
        for path_point in navi_path.path.path_point:
            navigation_path_x.append(path_point.x)
            navigation_path_y.append(path_point.y)


def draw_lane_central(lane, ax, color_val):
    """draw boundary"""
    for curve in lane.central_curve.segment:
        if curve.HasField('line_segment'):
            px = []
            py = []
            for p in curve.line_segment.point:
                px.append(float(p.y))
                py.append(float(p.x))
            ax.plot(px, py, ls=':', c=color_val, alpha=0.5)


def update(frame_number):
    plt.cla()
    if map_msg is not None:
        for lane in map_msg.hdmap.lane:
            draw_lane_boundary(lane, ax, 'b', map_msg.lane_marker)
            draw_lane_central(lane, ax, 'r')

        for key in map_msg.navigation_path:
            x = []
            y = []
            for point in map_msg.navigation_path[key].path.path_point:
                x.append(point.y)
                y.append(point.x)
            ax.plot(x, y, ls='-', c='g', alpha=0.3)

    if planning_msg is not None:
        x = []
        y = []
        for tp in planning_msg.trajectory_point:
            x.append(tp.path_point.y)
            y.append(tp.path_point.x)
        ax.plot(x, y, ls=':', c='r', linewidth=5.0)

    ax.axvline(x=0.0, alpha=0.3)
    ax.axhline(y=0.0, alpha=0.3)
    ax.set_xlim([10, -10])
    ax.set_ylim([-10, 200])
    y = 10
    while y < 200:
        ax.plot([10, -10], [y, y], ls='-', c='g', alpha=0.3)
        y = y + 10
    plt.yticks(np.arange(10, 200, 10))
    adc = plt.Circle((0, 0), 0.3, color='r')
    plt.gcf().gca().add_artist(adc)
    ax.relim()


def map_callback(map_msg_pb):
    global map_msg
    map_msg = map_msg_pb

def planning_callback(planning_msg_pb):
    global planning_msg
    planning_msg = planning_msg_pb


def add_listener():
    rospy.init_node('relative_map_plot', anonymous=True)
    rospy.Subscriber('/apollo/relative_map',
                     navigation_pb2.MapMsg,
                     map_callback)
    rospy.Subscriber('/apollo/navigation',
                     navigation_pb2.NavigationInfo, navigation_callback)
    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     localization_callback)
    rospy.Subscriber('/apollo/planning',
                     planning_pb2.ADCTrajectory,
                     planning_callback)


if __name__ == '__main__':
    add_listener()
    fig = plt.figure()
    ax = plt.subplot2grid((1, 1), (0, 0), rowspan=3, colspan=1)
    ani = animation.FuncAnimation(fig, update, interval=100)
    plt.show()
