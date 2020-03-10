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
import time
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import String
from numpy.polynomial.polynomial import polyval
from modules.drivers.proto import mobileye_pb2
from modules.planning.proto import planning_pb2
from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2
from path_decider import PathDecider
from speed_decider import SpeedDecider
from trajectory_generator import TrajectoryGenerator
from provider_mobileye import MobileyeProvider
from provider_chassis import ChassisProvider
from provider_localization import LocalizationProvider
from provider_routing import RoutingProvider

planning_pub = None
routing_debug_pub = None

PUB_NODE_NAME = "planning"
PUB_TOPIC = "/apollo/" + PUB_NODE_NAME
CRUISE_SPEED = 10  # m/s
ENABLE_FOLLOW = False
ENABLE_ROUTING_AID = False

f = open("benchmark.txt", "w")
path_decider = PathDecider()
speed_decider = SpeedDecider(CRUISE_SPEED)
traj_generator = TrajectoryGenerator()
mobileye_provider = MobileyeProvider()
chassis_provider = ChassisProvider()
localization_provider = LocalizationProvider()
routing_provider = RoutingProvider()

nx = []
ny = []
local_seg_x = []
local_seg_y = []

local_smooth_seg_x = []
local_smooth_seg_y = []

left_marker_x = []
left_marker_y = []

right_marker_x = []
right_marker_y = []


def routing_callback(routing_str):
    routing_provider.update(routing_str)


def localization_callback(localization_pb):
    localization_provider.update(localization_pb)


def mobileye_callback(mobileye_pb):
    global nx, ny, local_seg_x, local_seg_y
    global left_marker_x, left_marker_y
    global right_marker_x, right_marker_y
    global local_smooth_seg_x, local_smooth_seg_y
    global local_smooth_seg_x2, local_smooth_seg_y2

    mobileye_provider.update(mobileye_pb)
    mobileye_provider.process_obstacles()

    vx = localization_provider.localization_pb.pose.position.x
    vy = localization_provider.localization_pb.pose.position.y
    heading = localization_provider.localization_pb.pose.heading

    local_seg_x, local_seg_y = routing_provider.get_local_segment(vx, vy,
                                                                  heading)

    local_smooth_seg_x, local_smooth_seg_y = \
        routing_provider.get_local_segment_spline(vx, vy, heading)

    left_marker_coef = mobileye_provider.left_lm_coef
    left_marker_x = []
    left_marker_y = []
    for x in range(int(mobileye_provider.left_lane_marker_range)):
        y = polyval(x, left_marker_coef)
        left_marker_x.append(x)
        left_marker_y.append(-y)

    right_marker_coef = mobileye_provider.right_lm_coef
    right_marker_x = []
    right_marker_y = []
    for x in range(int(mobileye_provider.right_lane_marker_range)):
        y = polyval(x, right_marker_coef)
        right_marker_x.append(x)
        right_marker_y.append(-y)


def add_listener():
    global planning_pub, routing_debug_pub
    rospy.init_node(PUB_NODE_NAME, anonymous=True)
    rospy.Subscriber('/apollo/sensor/mobileye',
                     mobileye_pb2.Mobileye,
                     mobileye_callback)
    rospy.Subscriber('/apollo/localization/pose',
                     localization_pb2.LocalizationEstimate,
                     localization_callback)
    rospy.Subscriber('/apollo/navigation/routing',
                     String, routing_callback)

    planning_pub = rospy.Publisher(
        PUB_TOPIC, planning_pb2.ADCTrajectory, queue_size=1)
    routing_debug_pub = rospy.Publisher(
        '/apollo/navigation/routing/debug', String, queue_size=1)


def update(frame_number):
    line2.set_xdata(local_seg_x)
    line2.set_ydata(local_seg_y)
    line1.set_xdata(left_marker_x)
    line1.set_ydata(left_marker_y)
    line4.set_xdata(right_marker_x)
    line4.set_ydata(right_marker_y)
    line3.set_xdata(local_smooth_seg_x)
    line3.set_ydata(local_smooth_seg_y)


if __name__ == '__main__':
    DEBUG = False
    line1 = None
    line2 = None
    add_listener()

    fig = plt.figure()
    ax = plt.subplot2grid((1, 1), (0, 0), rowspan=1, colspan=1)
    line1, = ax.plot([0], [0], 'r-')
    line4, = ax.plot([0], [0], 'k-')
    line2, = ax.plot([0], [0], 'b-')
    line3, = ax.plot([0], [0], 'g--', lw=3)
    ani = animation.FuncAnimation(fig, update, interval=100)
    ax.axvline(x=0.0, alpha=0.3)
    ax.axhline(y=0.0, alpha=0.3)
    ax.set_xlim([-200, 200])
    ax.set_ylim([-10, 10])

    plt.show()
