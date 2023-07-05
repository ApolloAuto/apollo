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
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.control_msgs import control_cmd_pb2


INIT_ACC_DATA = []
INIT_T_DATA = []

begin_t = None
last_t = None
last_v = None
lock = threading.Lock()

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Planning plot data length")


def callback(chassis_pb):
    global INIT_ACC_DATA, INIT_T_DATA
    global begin_t, last_t, last_v

    if begin_t is None:
        begin_t = chassis_pb.header.timestamp_sec
        last_t = begin_t
    current_t = chassis_pb.header.timestamp_sec
    current_v = chassis_pb.speed_mps

    print(current_v)
    if abs(current_t - last_t) < 0.015:
        return

    lock.acquire()
    if last_t is not None and abs(current_t - last_t) > 1:
        begin_t = chassis_pb.header.timestamp_sec

        INIT_ACC_DATA = []
        INIT_T_DATA = []

        last_t = None
        last_v = None

    if last_t is not None and last_v is not None and current_t > last_t:
        INIT_T_DATA.append(current_t - begin_t)
        INIT_ACC_DATA.append((current_v - last_v) / (current_t - last_t))

    lock.release()

    last_t = current_t
    last_v = current_v


def listener():
    cyber.init()
    test_node = cyber.Node("chassis_acc_listener")
    test_node.create_reader("/apollo/canbus/chassis",
                            chassis_pb2.Chassis, callback)


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    lock.acquire()
    init_data_line.set_xdata(INIT_T_DATA)
    init_data_line.set_ydata(INIT_ACC_DATA)
    lock.release()
    #brake_text.set_text('brake = %.1f' % brake_data[-1])
    #throttle_text.set_text('throttle = %.1f' % throttle_data[-1])
    if len(INIT_ACC_DATA) > 0:
        init_data_text.set_text('chassis acc = %.1f' % INIT_ACC_DATA[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    listener()
    fig, ax = plt.subplots()
    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()
    init_data_line, = ax.plot(
        INIT_T_DATA, INIT_ACC_DATA, 'b', lw=2, alpha=0.7, label='chassis acc')

    #brake_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    #throttle_text = ax.text(0.75, 0.90, '', transform=ax.transAxes)
    init_data_text = ax.text(0.75, 0.95, '', transform=ax.transAxes)
    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.set_ylim(-6, 3)
    ax.set_xlim(-1, 60)
    ax.legend(loc="upper left")
    plt.show()
