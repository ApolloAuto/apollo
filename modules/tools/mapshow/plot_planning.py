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

import os
import rospy
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from map import Map
from localization import Localization
from modules.planning.proto import planning_pb2
from modules.localization.proto import localization_pb2
import copy

PATH_DATA_X = {}
PATH_DATA_Y = {}
path_line_pool = []
path_line_pool_size = 4

SPEED_DATA_TIME = {}
SPEED_DATA_VAL = {}
speed_line_pool = []
speed_line_pool_size = 4

vehicle_position_line = None
vehicle_polygon_line = None
localization_pb_buff = None


def localization_callback(localization_pb):
    global localization_pb_buff
    localization_pb_buff = localization_pb


def planning_callback(planning_pb):
    global PATH_DATA_X, PATH_DATA_Y
    global SPEED_DATA_TIME, SPEED_DATA_VAL
    # path
    path_data_x = {}
    path_data_y = {}
    for path_debug in planning_pb.debug.planning_data.path:
        name = path_debug.name
        path_data_x[name] = []
        path_data_y[name] = []
        for path_point in path_debug.path_point:
            path_data_x[name].append(path_point.x)
            path_data_y[name].append(path_point.y)
    PATH_DATA_X = copy.deepcopy(path_data_x)
    PATH_DATA_Y = copy.deepcopy(path_data_y)

    # speed
    speed_data_time = {}
    speed_data_val = {}
    for speed_plan in planning_pb.debug.planning_data.speed_plan:
        name = speed_plan.name
        speed_data_time[name] = []
        speed_data_val[name] = []
        for speed_point in speed_plan.speed_point:
            speed_data_time[name].append(speed_point.t)
            speed_data_val[name].append(speed_point.v)
    name = "final_output"
    speed_data_time[name] = []
    speed_data_val[name] = []
    for traj_point in planning_pb.trajectory_point:
        speed_data_time[name].append(traj_point.relative_time)
        speed_data_val[name].append(traj_point.v)
    SPEED_DATA_TIME = speed_data_time
    SPEED_DATA_VAL = speed_data_val


def add_listener():
    rospy.init_node('plot_planning', anonymous=True)
    rospy.Subscriber('/apollo/planning',
                     planning_pb2.ADCTrajectory,
                     planning_callback)
    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     localization_callback)


def update(frame_number):
    update_init()
    update_path()
    update_speed()
    update_localization()
    ax.relim()
    ax.autoscale_view()
    ax.legend(loc="upper left")
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, 1.05))


def update_init():
    for line in speed_line_pool:
        line.set_visible(False)
        line.set_label(None)
    for line in path_line_pool:
        line.set_visible(False)
        line.set_label(None)
    vehicle_position_line.set_visible(False)
    vehicle_polygon_line.set_visible(False)


def update_speed():
    cnt = 0
    for name in SPEED_DATA_TIME.keys():
        if cnt >= len(speed_line_pool):
            print "Number of lines is more than 4! "
            continue
        line = speed_line_pool[cnt]
        line.set_visible(True)
        if len(SPEED_DATA_TIME[name]) <= 1:
            continue
        line.set_xdata(SPEED_DATA_TIME[name])
        line.set_ydata(SPEED_DATA_VAL[name])
        line.set_label(name)
        cnt += 1


def update_path():
    cnt = 0
    for name in PATH_DATA_X.keys():
        if cnt >= len(path_line_pool):
            print "Number of lines is more than 4! "
            continue
        line = path_line_pool[cnt]
        line.set_visible(True)
        if len(PATH_DATA_X[name]) <= 1:
            continue
        line.set_xdata(PATH_DATA_X[name])
        line.set_ydata(PATH_DATA_Y[name])
        line.set_label(name)
        cnt += 1


def update_localization():
    if localization_pb_buff is not None:
        vehicle_position_line.set_visible(True)
        vehicle_polygon_line.set_visible(True)
        Localization(localization_pb_buff) \
            .replot_vehicle(vehicle_position_line, vehicle_polygon_line)


def init_line_pool(central_x, central_y):
    global vehicle_position_line, vehicle_polygon_line, s_speed_line
    colors = ['b', 'g', 'r', 'k']

    for i in range(path_line_pool_size):
        line, = ax.plot([central_x], [central_y],
                        colors[i % len(colors)], lw=3, alpha=0.5)
        path_line_pool.append(line)

    for i in range(speed_line_pool_size):
        line, = ax1.plot([central_x], [central_y],
                        colors[i % len(colors)]+".", lw=3, alpha=0.5)
        speed_line_pool.append(line)

    vehicle_position_line, = ax.plot([central_x], [central_y], 'go', alpha=0.3)
    vehicle_polygon_line, = ax.plot([central_x], [central_y], 'g-')


if __name__ == '__main__':
    default_map_path = os.path.dirname(os.path.realpath(__file__))
    default_map_path += "/../../map/data/base_map.txt"

    parser = argparse.ArgumentParser(
        description="plot_planning is a tool to display planning trajs on a map.",
        prog="plot_planning.py")
    parser.add_argument(
        "-m", "--map", action="store", type=str, required=False, default=default_map_path,
        help="Specify the map file in txt or binary format")
    args = parser.parse_args()

    add_listener()
    fig = plt.figure()
    ax = plt.subplot2grid((2, 3), (0, 0), rowspan=2, colspan=2)
    ax1 = plt.subplot2grid((2, 3), (0, 2))
    ax1.set_xlabel("t (second)")
    ax1.set_xlim([-2, 10])
    ax1.set_ylim([-1, 10])
    ax1.set_ylabel("speed (m/s)")

    map = Map()
    map.load(args.map)
    map.draw_lanes(ax, False, [])
    central_y = sum(ax.get_ylim())/2
    central_x = sum(ax.get_xlim())/2

    init_line_pool(central_x, central_y)

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.legend(loc="upper left")
    ax.axis('equal')
    plt.show()
