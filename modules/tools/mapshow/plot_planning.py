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

import sys
import rospy
import gflags
from gflags import FLAGS
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from map import Map
import threading
from modules.planning.proto import planning_pb2

DATAX = {}
DATAY = {}

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Control plot data length")

PATHLOCK = threading.Lock()
line_pool = []
line_pool_size = 4


def callback(data):
    global DATAX, DATAY
    PATHLOCK.acquire()
    planning_pb = planning_pb2.ADCTrajectory()
    planning_pb.CopyFrom(data)
    DATAX = {}
    DATAY = {}
    for path_debug in planning_pb.debug.planning_data.path:
        name = path_debug.name
        DATAX[name] = []
        DATAY[name] = []
        for path_point in path_debug.path.path_point:
            DATAX[name].append(path_point.x)
            DATAY[name].append(path_point.y)
        #print len(DATAX[name])
        #print len(DATAY[name])
    PATHLOCK.release()


def listener():
    rospy.init_node('plot_planning', anonymous=True)
    rospy.Subscriber('/apollo/planning',
                     planning_pb2.ADCTrajectory,
                     callback)


def update(frame_number):
    PATHLOCK.acquire()
    for line in line_pool:
        line.set_visible(False)
        line.set_label(None)
    cnt = 0
    for name in DATAX.keys():
        if cnt >= len(line_pool):
            print "Number of lines is more than 4! "
            continue
        line = line_pool[cnt]
        line.set_visible(True)
        line.set_xdata(DATAX[name])
        line.set_ydata(DATAY[name])
        line.set_label(name)
        cnt += 1
    PATHLOCK.release()

    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()
    ax.legend(loc="upper left")


def init_line_pool(central_x, central_y):
    colors = ['b', 'g', 'r', 'k']
    for i in range(line_pool_size):
        line, = ax.plot([central_x], [central_y],
                        colors[i % len(colors)], lw=3, alpha=0.5)
        line_pool.append(line)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    listener()
    fig, ax = plt.subplots()

    map = Map()
    map.load("../../map/data/base_map.txt")
    map.draw_lanes(ax, False, [])
    central_y = sum(ax.get_ylim())/2
    central_x = sum(ax.get_xlim())/2

    init_line_pool(central_x, central_y)

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.legend(loc="upper left")
    ax.axis('equal')
    plt.show()
