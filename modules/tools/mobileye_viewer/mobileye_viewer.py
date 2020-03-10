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
import json
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from modules.planning.proto import planning_pb2
from modules.drivers.proto import mobileye_pb2
from mobileye_data import MobileyeData
from localization_data import LocalizationData
from planning_data import PlanningData
from routing_data import RoutingData
from chassis_data import ChassisData
from view_subplot import ViewSubplot
from subplot_s_speed import SubplotSSpeed
from subplot_s_theta import SubplotSTheta
from subplot_s_time import SubplotSTime
from modules.localization.proto import localization_pb2
from modules.map.relative_map.proto import navigation_pb2
from modules.canbus.proto import chassis_pb2
from std_msgs.msg import String

PLANNING_TOPIC = '/apollo/planning'
mobileye = MobileyeData()
localization = LocalizationData()
planning = PlanningData()
chassis = ChassisData()
routing_data = RoutingData()


def update(frame_number):
    view_subplot.show(mobileye, localization, planning, chassis, routing_data)
    s_speed_subplot.show(planning)
    s_theta_subplot.show(planning)
    s_time_subplot.show(planning)


def localization_callback(localization_pb):
    localization.update(localization_pb)


def mobileye_callback(mobileye_pb):
    mobileye.update(mobileye_pb)
    mobileye.compute_lanes()
    # mobileye.compute_next_lanes()
    mobileye.compute_obstacles()
    planning.compute_path()
    planning.compute_path_param()


def planning_callback(planning_pb):
    planning.update(planning_pb)


def chassis_callback(chassis_pb):
    chassis.update(chassis_pb)


def routing_callback(navigation_info_pb):
    routing_data.update_navigation(navigation_info_pb)


def add_listener():
    rospy.init_node('mobileye_plot', anonymous=True)
    rospy.Subscriber('/apollo/sensor/mobileye',
                     mobileye_pb2.Mobileye,
                     mobileye_callback)
    rospy.Subscriber(PLANNING_TOPIC,
                     planning_pb2.ADCTrajectory,
                     planning_callback)
    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     localization_callback)
    rospy.Subscriber('/apollo/canbus/chassis',
                     chassis_pb2.Chassis,
                     chassis_callback)
    rospy.Subscriber('/apollo/navigation',
                     navigation_pb2.NavigationInfo, routing_callback)


if __name__ == '__main__':
    add_listener()
    fig = plt.figure()

    ax = plt.subplot2grid((3, 2), (0, 0), rowspan=3, colspan=1)
    view_subplot = ViewSubplot(ax)

    ax1 = plt.subplot2grid((3, 2), (0, 1), rowspan=1, colspan=1)
    s_speed_subplot = SubplotSSpeed(ax1)

    ax2 = plt.subplot2grid((3, 2), (1, 1), rowspan=1, colspan=1)
    s_theta_subplot = SubplotSTheta(ax2)

    ax3 = plt.subplot2grid((3, 2), (2, 1), rowspan=1, colspan=1)
    s_time_subplot = SubplotSTime(ax3)

    ani = animation.FuncAnimation(fig, update, interval=100)

    # ax.axis('equal')
    ax.axvline(x=0.0, alpha=0.3)
    ax.axhline(y=0.0, alpha=0.3)
    ax2.axvline(x=0.0, alpha=0.3)
    ax2.axhline(y=0.0, alpha=0.3)
    plt.show()
