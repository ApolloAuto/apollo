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

import sys
import gflags
from cyber.python.cyber_py3 import cyber
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from modules.common_msgs.control_msgs import control_cmd_pb2
BRAKE_LINE_DATA = []
TROTTLE_LINE_DATA = []
STEERING_LINE_DATA = []

FLAGS = gflags.FLAGS
gflags.DEFINE_integer("data_length", 500, "Control plot data length")


def callback(control_cmd_pb):
    global STEERING_LINE_DATA
    global TROTTLE_LINE_DATA, BRAKE_LINE_DATA

    STEERING_LINE_DATA.append(control_cmd_pb.steering_target)
    if len(STEERING_LINE_DATA) > FLAGS.data_length:
        STEERING_LINE_DATA = STEERING_LINE_DATA[-FLAGS.data_length:]

    BRAKE_LINE_DATA.append(control_cmd_pb.brake)
    if len(BRAKE_LINE_DATA) > FLAGS.data_length:
        BRAKE_LINE_DATA = BRAKE_LINE_DATA[-FLAGS.data_length:]

    TROTTLE_LINE_DATA.append(control_cmd_pb.throttle)
    if len(TROTTLE_LINE_DATA) > FLAGS.data_length:
        TROTTLE_LINE_DATA = TROTTLE_LINE_DATA[-FLAGS.data_length:]


def listener():
    cyber.init()
    test_node = cyber.Node("control_listener")
    test_node.create_reader("/apollo/control",
                            control_cmd_pb2.ControlCommand, callback)
    test_node.spin()
    cyber.shutdown()


def compensate(data_list):
    comp_data = [0] * FLAGS.data_length
    comp_data.extend(data_list)
    if len(comp_data) > FLAGS.data_length:
        comp_data = comp_data[-FLAGS.data_length:]
    return comp_data


def update(frame_number):
    brake_data = compensate(BRAKE_LINE_DATA)
    brake_line.set_ydata(brake_data)

    throttle_data = compensate(TROTTLE_LINE_DATA)
    throttle_line.set_ydata(throttle_data)

    steering_data = compensate(STEERING_LINE_DATA)
    steering_line.set_ydata(steering_data)

    brake_text.set_text('brake = %.1f' % brake_data[-1])
    throttle_text.set_text('throttle = %.1f' % throttle_data[-1])
    steering_text.set_text('steering = %.1f' % steering_data[-1])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    listener()
    fig, ax = plt.subplots()
    X = range(FLAGS.data_length)
    Xs = [i * -1 for i in X]
    Xs.sort()
    steering_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'b', lw=3, alpha=0.5, label='steering')
    throttle_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'g', lw=3, alpha=0.5, label='throttle')
    brake_line, = ax.plot(
        Xs, [0] * FLAGS.data_length, 'r', lw=3, alpha=0.5, label='brake')
    brake_text = ax.text(0.75, 0.85, '', transform=ax.transAxes)
    throttle_text = ax.text(0.75, 0.90, '', transform=ax.transAxes)
    steering_text = ax.text(0.75, 0.95, '', transform=ax.transAxes)
    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.set_ylim(-100, 120)
    ax.set_xlim(-1 * FLAGS.data_length, 10)
    ax.legend(loc="upper left")
    plt.show()
