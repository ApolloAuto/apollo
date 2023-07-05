#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
import threading

import gflags
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.planning_msgs import planning_pb2


LAST_TRAJ_DATA = []
LAST_TRAJ_T_DATA = []
CURRENT_TRAJ_DATA = []
CURRENT_TRAJ_T_DATA = []
INIT_V_DATA = []
INIT_T_DATA = []

begin_t = None
last_t = None
last_v = None
lock = threading.Lock()

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Planning plot data length")


def callback(planning_pb):
    global INIT_V_DATA, INIT_T_DATA
    global CURRENT_TRAJ_DATA, LAST_TRAJ_DATA
    global CURRENT_TRAJ_T_DATA, LAST_TRAJ_T_DATA
    global begin_t, last_t, last_v

    lock.acquire()

    if begin_t is None:
        begin_t = planning_pb.header.timestamp_sec
    current_t = planning_pb.header.timestamp_sec
    current_v = planning_pb.debug.planning_data.init_point.v

    if last_t is not None and abs(current_t - last_t) > 1:
        begin_t = planning_pb.header.timestamp_sec
        LAST_TRAJ_DATA = []
        LAST_TRAJ_T_DATA = []

        CURRENT_TRAJ_DATA = []
        CURRENT_TRAJ_T_DATA = []

        INIT_V_DATA = []
        INIT_T_DATA = []

        last_t = None
        last_v = None

    if last_t is not None and last_v is not None and current_t > last_t:
        INIT_T_DATA.append(current_t - begin_t)
        INIT_V_DATA.append((current_v - last_v) / (current_t - last_t))

        LAST_TRAJ_DATA = []
        for v in CURRENT_TRAJ_DATA:
            LAST_TRAJ_DATA.append(v)

        LAST_TRAJ_T_DATA = []
        for t in CURRENT_TRAJ_T_DATA:
            LAST_TRAJ_T_DATA.append(t)

        CURRENT_TRAJ_DATA = []
        CURRENT_TRAJ_T_DATA = []
        traj_point_last_v = None
        traj_point_last_t = None
        for traj_point in planning_pb.trajectory_point:
            if traj_point_last_v is None:
                CURRENT_TRAJ_DATA.append(traj_point.a)
            else:
                # CURRENT_TRAJ_DATA.append(traj_point.a)
                cal_a = (traj_point.v - traj_point_last_v) / \
                    (traj_point.relative_time - traj_point_last_t)
                CURRENT_TRAJ_DATA.append(cal_a)
            CURRENT_TRAJ_T_DATA.append(current_t - begin_t + traj_point.relative_time)
            traj_point_last_t = traj_point.relative_time
            traj_point_last_v = traj_point.v
    lock.release()

    last_t = current_t
    last_v = current_v


def listener():
    cyber.init()
    test_node = cyber.Node("planning_acc_listener")
    test_node.create_reader("/apollo/planning",
                            planning_pb2.ADCTrajectory, callback)


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    lock.acquire()
    last_traj.set_xdata(LAST_TRAJ_T_DATA)
    last_traj.set_ydata(LAST_TRAJ_DATA)

    current_traj.set_xdata(CURRENT_TRAJ_T_DATA)
    current_traj.set_ydata(CURRENT_TRAJ_DATA)

    init_data_line.set_xdata(INIT_T_DATA)
    init_data_line.set_ydata(INIT_V_DATA)
    lock.release()
    #brake_text.set_text('brake = %.1f' % brake_data[-1])
    #throttle_text.set_text('throttle = %.1f' % throttle_data[-1])
    if len(INIT_V_DATA) > 0:
        init_data_text.set_text('init point a = %.1f' % INIT_V_DATA[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    listener()
    fig, ax = plt.subplots()
    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()
    init_data_line, = ax.plot(
        INIT_T_DATA, INIT_V_DATA, 'b', lw=2, alpha=0.7, label='init_point_a')
    current_traj, = ax.plot(
        CURRENT_TRAJ_T_DATA, CURRENT_TRAJ_DATA, 'r', lw=1, alpha=0.5, label='current_traj')
    last_traj, = ax.plot(
        LAST_TRAJ_T_DATA, LAST_TRAJ_DATA, 'g', lw=1, alpha=0.5, label='last_traj')

    #brake_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    #throttle_text = ax.text(0.75, 0.90, '', transform=ax.transAxes)
    init_data_text = ax.text(0.75, 0.95, '', transform=ax.transAxes)
    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.set_ylim(-6, 3)
    ax.set_xlim(-1, 60)
    ax.legend(loc="upper left")
    plt.show()
