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

import matplotlib.animation as animation
import matplotlib.pyplot as plt

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.planning_msgs import planning_pb2
from modules.tools.mapshow.libs.planning import Planning
from modules.tools.mapshow.libs.subplot_traj_acc import TrajAccSubplot
from modules.tools.mapshow.libs.subplot_traj_path import TrajPathSubplot
from modules.tools.mapshow.libs.subplot_traj_speed import TrajSpeedSubplot


planning = Planning()


def update(frame_number):
    traj_speed_subplot.show(planning)
    traj_acc_subplot.show(planning)
    traj_path_subplot.show(planning)


def planning_callback(planning_pb):
    planning.update_planning_pb(planning_pb)
    planning.compute_traj_data()


def add_listener():
    planning_sub = cyber.Node("st_plot")
    planning_sub.create_reader('/apollo/planning', planning_pb2.ADCTrajectory,
                               planning_callback)


def press_key():
    pass


if __name__ == '__main__':
    cyber.init()
    add_listener()
    fig = plt.figure(figsize=(14, 6))
    fig.canvas.mpl_connect('key_press_event', press_key)

    ax = plt.subplot2grid((2, 2), (0, 0))
    traj_speed_subplot = TrajSpeedSubplot(ax)

    ax2 = plt.subplot2grid((2, 2), (0, 1))
    traj_acc_subplot = TrajAccSubplot(ax2)

    ax3 = plt.subplot2grid((2, 2), (1, 0))
    traj_path_subplot = TrajPathSubplot(ax3)

    ani = animation.FuncAnimation(fig, update, interval=100)

    plt.show()
    cyber.shutdown()
