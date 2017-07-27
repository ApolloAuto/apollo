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

from modules.planning.proto import planning_pb2

REF_LINE_X = []
REF_LINE_Y = []
DP_LINE_X = []
DP_LINE_Y = []
QP_LINE_X = []
QP_LINE_Y = []

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 80, "Control plot data length")


def callback(data):
    #print "callback()"
    global REF_LINE_X, REF_LINE_Y
    global DP_LINE_X, DP_LINE_Y
    global QP_LINE_X, QP_LINE_Y
    planning_pb = planning_pb2.ADCTrajectory()
    planning_pb.CopyFrom(data)

    for path_debug in planning_pb.debug.planning_data.path:
        REF_LINE_X = []
        REF_LINE_Y = []
        DP_LINE_X = []
        DP_LINE_Y = []
        QP_LINE_X = []
        QP_LINE_Y = []
        #TODO(yifei) path name need to be updated
        if path_debug.name == "planning_reference_line":
            for path_point in path_debug.path:
                REF_LINE_X.append(path_point.x)
                REF_LINE_Y.append(path_point.y)
        if path_debug.name == "dp":
            for path_point in path_debug.path:
                DP_LINE_X.append(path_point.x)
                DP_LINE_Y.append(path_point.y)
        if path_debug.name == "qp":
            for path_point in path_debug.path:
                QP_LINE_X.append(path_point.x)
                QP_LINE_Y.append(path_point.y)


def listener():
    rospy.init_node('plot_planning', anonymous=True)
    rospy.Subscriber('/apollo/planning',
                     planning_pb2.ADCTrajectory,
                     callback)


def compensate(data_list):
    if len(data_list) > FLAGS.data_length:
        data_list = data_list[0:FLAGS.data_length]
    else:
        for i in range(FLAGS.data_length - len(data_list)):
            data_list.append(data_list[-1])
    return data_list


def update(frame_number):
    #print frame_number, len(REF_LINE_X)
    if len(REF_LINE_X) == 0:
        ref_line.set_visible(False)
    else:
        ref_line.set_visible(True)

        ref_line_x = compensate(REF_LINE_X)
        ref_line.set_xdata(ref_line_x)

        ref_line_y = compensate(REF_LINE_Y)
        ref_line.set_ydata(ref_line_y)

    if len(DP_LINE_X) == 0:
        dp_line.set_visible(False)
    else:
        dp_line.set_visible(True)

        dp_line_x = compensate(DP_LINE_X)
        dp_line.set_xdata(dp_line_x)

        dp_line_y = compensate(DP_LINE_Y)
        dp_line.set_ydata(dp_line_y)

    if len(QP_LINE_X) == 0:
        qp_line.set_visible(False)
    else:
        qp_line.set_visible(True)

        qp_line_x = compensate(QP_LINE_X)
        qp_line.set_xdata(qp_line_x)

        qp_line_y = compensate(QP_LINE_Y)
        qp_line.set_ydata(qp_line_y)
    ax.relim()
    # update ax.viewLim using the new dataLim
    ax.autoscale_view()


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    #listener()
    fig, ax = plt.subplots()
    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()

    map = Map()
    map.load("../../map/data/base_map.txt")
    map.draw_lanes(ax, False, [])
    central_y = sum(ax.get_ylim())/2
    central_x = sum(ax.get_xlim())/2

    ref_line, = ax.plot(
        [central_x] * FLAGS.data_length, [central_y] * FLAGS.data_length,
        'b', lw=3, alpha=0.5, label='reference')
    dp_line, = ax.plot(
        [central_x] * FLAGS.data_length, [central_y] * FLAGS.data_length,
        'g', lw=3, alpha=0.5, label='dp path')
    qp_line, = ax.plot(
        [central_x] * FLAGS.data_length, [central_y] * FLAGS.data_length,
        'r', lw=3, alpha=0.5, label='qp path')

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.legend(loc="upper left")
    ax.axis('equal')
    plt.show()
