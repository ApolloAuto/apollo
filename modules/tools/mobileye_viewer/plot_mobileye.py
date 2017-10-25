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
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from modules.drivers.proto import mobileye_pb2
from mobileye_data import MobileyeData
from view_subplot import ViewSubplot

mobileye = MobileyeData()

def update(frame_number):
    view_subplot.show(mobileye)


def mobileye_callback(mobileye_pb):
    mobileye.update(mobileye_pb)
    mobileye.compute_lanes()
    mobileye.compute_obstacles()


def add_listener():
    rospy.init_node('mobileye_plot', anonymous=True)
    rospy.Subscriber('/apollo/mobileye', mobileye_pb2.Mobileye,
                     mobileye_callback)


if __name__ == '__main__':

    add_listener()
    fig = plt.figure()

    ax = plt.subplot2grid((1, 1), (0, 0), rowspan=2, colspan=2)

    view_subplot = ViewSubplot(ax)

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.axis('equal')
    plt.show()
