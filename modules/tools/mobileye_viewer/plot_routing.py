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

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import String
from routing_data import RoutingData
from subplot_routing import SubplotRouting

routing_data = RoutingData()


def update(frame_number):
    subplot_routing.show(routing_data)


def routing_callback(routing_str):
    routing_data.update(routing_str)


def routing_debug_callback(routing_debug_str):
    if routing_debug_str is not None:
        routing_data.update_debug(routing_debug_str)


def add_listener():
    rospy.init_node('navi_routing_plot', anonymous=True)
    rospy.Subscriber('/apollo/navigation/routing',
                     String, routing_callback)
    rospy.Subscriber('/apollo/navigation/routing/debug',
                     String, routing_debug_callback)


if __name__ == '__main__':
    add_listener()
    fig = plt.figure()

    ax = plt.subplot2grid((1, 1), (0, 0), rowspan=1, colspan=1)
    subplot_routing = SubplotRouting(ax)

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.axis('equal')
    plt.show()
