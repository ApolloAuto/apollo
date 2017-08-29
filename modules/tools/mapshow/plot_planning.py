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

import argparse
import os
import threading

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import rospy

from localization import Localization
from map import Map
from modules.planning.proto import planning_pb2
from modules.localization.proto import localization_pb2
from planning import Planning

planning = Planning()
localization = Localization()

path_line_pool = []
path_line_pool_size = 4

speed_line_pool = []
speed_line_pool_size = 4

vehicle_position_line = None
vehicle_polygon_line = None

st_line_1 = None
st_line_2 = None

obstacle_line_pool = []
obstacle_annotation_pool = []
obstacle_line_pool_size = 10

obstacle_line_pool2 = []
obstacle_annotation_pool2 = []
obstacle_line_pool_size2 = 10

sl_static_obstacle_lower_boundary = None
sl_static_obstacle_upper_boundary = None
sl_dynamic_obstacle_lower_boundary = None
sl_dynamic_obstacle_upper_boundary = None
sl_map_lower_boundary = None
sl_map_upper_boundary = None
sl_path = None
sl_aggregated_boundary_low_line = None
sl_aggregated_boundary_high_line = None


def planning_callback(planning_pb):
    planning.update_planning_pb(planning_pb)
    planning.compute_path_data()
    planning.compute_speed_data()
    planning.compute_st_data()
    planning.compute_sl_data()
    localization.update_localization_pb(
        planning_pb.debug.planning_data.adc_position)


def add_listener():
    rospy.init_node('plot_planning', anonymous=True)
    rospy.Subscriber('/apollo/planning', planning_pb2.ADCTrajectory,
                     planning_callback)


def update(frame_number):
    for line in speed_line_pool:
        line.set_visible(False)
        line.set_label(None)
    for line in path_line_pool:
        line.set_visible(False)
        line.set_label(None)
    st_line_1.set_visible(False)
    st_line_1.set_label(None)
    st_line_2.set_visible(False)
    st_line_2.set_label(None)
    for line in obstacle_line_pool:
        line.set_visible(False)
    for line in obstacle_annotation_pool:
        line.set_visible(False)
    for line in obstacle_line_pool2:
        line.set_visible(False)
    for line in obstacle_annotation_pool2:
        line.set_visible(False)
    sl_static_obstacle_lower_boundary.set_visible(False)
    sl_static_obstacle_upper_boundary.set_visible(False)
    sl_dynamic_obstacle_lower_boundary.set_visible(False)
    sl_dynamic_obstacle_upper_boundary.set_visible(False)
    sl_map_lower_boundary.set_visible(False)
    sl_map_upper_boundary.set_visible(False)
    sl_path.set_visible(False)
    sl_aggregated_boundary_low_line.set_visible(False)
    sl_aggregated_boundary_high_line.set_visible(False)

    vehicle_position_line.set_visible(False)
    vehicle_polygon_line.set_visible(False)

    planning.replot_path_data(path_line_pool)
    planning.replot_speed_data(speed_line_pool)
    planning.replot_sl_data(
        sl_static_obstacle_lower_boundary, sl_static_obstacle_upper_boundary,
        sl_dynamic_obstacle_lower_boundary, sl_dynamic_obstacle_upper_boundary,
        sl_map_lower_boundary, sl_map_upper_boundary, sl_path,
        sl_aggregated_boundary_low_line, sl_aggregated_boundary_high_line)

    if len(planning.st_data_s.keys()) >= 1:
        planning.replot_st_data(obstacle_line_pool, st_line_1,
                                obstacle_annotation_pool,
                                planning.st_data_s.keys()[0])
    if len(planning.st_data_s.keys()) >= 2:
        planning.replot_st_data(obstacle_line_pool2, st_line_2,
                                obstacle_annotation_pool2,
                                planning.st_data_s.keys()[1])
    localization.replot_vehicle(vehicle_position_line, vehicle_polygon_line)
    try:
        ax.set_xlim(localization.localization_pb.pose.position.x - 60,
                    localization.localization_pb.pose.position.x + 60)
        ax.set_ylim(localization.localization_pb.pose.position.y - 60,
                    localization.localization_pb.pose.position.y + 60)
    except:
        pass
    ax.relim()
    ax.autoscale_view()
    ax.legend(loc="upper left")
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, 1.1), ncol=5)
    ax2.legend(loc="upper center", bbox_to_anchor=(0.5, 1.1), ncol=5)
    ax3.legend(loc="upper center", bbox_to_anchor=(0.5, 1.1), ncol=5)
    return obstacle_annotation_pool[0]


def init_line_pool(central_x, central_y):
    global vehicle_position_line, vehicle_polygon_line, s_speed_line
    global obstacle_line_pool, st_line_1, st_line_2
    global sl_static_obstacle_lower_boundary
    global sl_static_obstacle_upper_boundary
    global sl_dynamic_obstacle_lower_boundary
    global sl_dynamic_obstacle_upper_boundary
    global sl_map_lower_boundary
    global sl_map_upper_boundary, sl_path
    global sl_aggregated_boundary_low_line, sl_aggregated_boundary_high_line

    colors = ['b', 'g', 'r', 'k']

    for i in range(path_line_pool_size):
        line, = ax.plot(
            [central_x], [central_y],
            colors[i % len(colors)],
            lw=3 + i * 3,
            alpha=0.2)
        path_line_pool.append(line)

    for i in range(speed_line_pool_size):
        line, = ax1.plot(
            [central_x], [central_y],
            colors[i % len(colors)] + ".",
            lw=3,
            alpha=0.5)
        speed_line_pool.append(line)

    st_line_1, = ax2.plot(
        [0], [0], colors[i % len(colors)] + ".", lw=3, alpha=0.5)
    st_line_2, = ax3.plot(
        [0], [0], colors[i % len(colors)] + ".", lw=3, alpha=0.5)

    sl_static_obstacle_lower_boundary, = \
        ax4.plot([0], [0], "r-", lw=0.3, alpha=0.8)
    sl_static_obstacle_upper_boundary, = \
        ax4.plot([0], [0], "r-", lw=0.3, alpha=0.8)
    sl_dynamic_obstacle_lower_boundary, = \
        ax4.plot([0], [0], "y-", lw=0.3, alpha=0.8)
    sl_dynamic_obstacle_upper_boundary, = \
        ax4.plot([0], [0], "y-", lw=0.3, alpha=0.8)
    sl_map_lower_boundary, = \
        ax4.plot([0], [0], "b-", lw=0.3, ms=2, alpha=0.8)
    sl_map_upper_boundary, = \
        ax4.plot([0], [0], "b-", lw=0.3, ms=4, alpha=0.8)
    sl_path, = ax4.plot([0], [0], "k--")
    sl_aggregated_boundary_low_line, = \
        ax4.plot([0], [0], "k-", lw=1, ms=2)
    sl_aggregated_boundary_high_line, = \
        ax4.plot([0], [0], "k-", lw=1, ms=2)

    for i in range(obstacle_line_pool_size):
        line, = ax2.plot([0], [0], 'r-', lw=3, alpha=0.5)
        anno = ax2.text(0, 0, "")
        obstacle_line_pool.append(line)
        obstacle_annotation_pool.append(anno)

    for i in range(obstacle_line_pool_size2):
        line, = ax3.plot([0], [0], 'r-', lw=3, alpha=0.5)
        anno = ax3.text(0, 0, "")
        obstacle_line_pool2.append(line)
        obstacle_annotation_pool2.append(anno)

    ax2.set_xlim(-3, 9)
    ax2.set_ylim(-10, 90)
    ax3.set_xlim(-3, 9)
    ax3.set_ylim(-10, 90)

    vehicle_position_line, = ax.plot([central_x], [central_y], 'go', alpha=0.3)
    vehicle_polygon_line, = ax.plot([central_x], [central_y], 'g-')


if __name__ == '__main__':
    default_map_path = os.path.dirname(os.path.realpath(__file__))
    default_map_path += "/../../map/data/sunnyvale_loop/sim_map.bin"

    parser = argparse.ArgumentParser(
        description="plot_planning is a tool to display "
        "planning trajs on a map.",
        prog="plot_planning.py")
    parser.add_argument(
        "-m",
        "--map",
        action="store",
        type=str,
        required=False,
        default=default_map_path,
        help="Specify the map file in txt or binary format")
    args = parser.parse_args()

    add_listener()
    fig = plt.figure()
    ax = plt.subplot2grid((3, 3), (0, 0), rowspan=2, colspan=2)
    ax1 = plt.subplot2grid((3, 3), (0, 2))
    ax2 = plt.subplot2grid((3, 3), (1, 2))
    ax3 = plt.subplot2grid((3, 3), (2, 2))

    ax4 = plt.subplot2grid((3, 3), (2, 0), colspan=2)
    ax4.set_xlim([30, 140])
    ax4.set_ylim([-5, 5])

    ax1.set_xlabel("t (second)")
    ax1.set_xlim([-2, 10])
    ax1.set_ylim([-1, 10])
    ax1.set_ylabel("speed (m/s)")

    map = Map()
    map.load(args.map)
    map.draw_lanes(ax, False, [])
    central_y = sum(ax.get_ylim()) / 2
    central_x = sum(ax.get_xlim()) / 2

    init_line_pool(central_x, central_y)

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.legend(loc="upper left")
    ax.axis('equal')
    plt.show()
