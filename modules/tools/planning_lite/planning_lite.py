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
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy.polynomial.polynomial as poly
from modules.drivers.proto import mobileye_pb2
from modules.planning.proto import planning_pb2
from modules.localization.proto import localization_pb2
from modules.canbus.proto import chassis_pb2

planning_pub = None
PUB_NODE_NAME = "planning"
PUB_TOPIC = "/apollo/" + PUB_NODE_NAME
LAST_INIT_LAT = None
MAX_LAT_CHANGE_PER_CYCLE = 0.1
f = open("benchmark.txt", "w")
SPEED = 0  # m/s
CRUISE_SPEED = 10  # m/s
MINIMUM_PATH_LENGTH = 5  # meter
x = []
y = []
nx = []
ny = []


def get_central_line_offset(current_init_lat):
    if LAST_INIT_LAT is None:
        return 0
    if abs(current_init_lat - LAST_INIT_LAT) < MAX_LAT_CHANGE_PER_CYCLE:
        return 0
    else:
        if current_init_lat > LAST_INIT_LAT:
            return - (
            abs(current_init_lat - LAST_INIT_LAT) - MAX_LAT_CHANGE_PER_CYCLE)
        else:
            return abs(
                current_init_lat - LAST_INIT_LAT) - MAX_LAT_CHANGE_PER_CYCLE


def get_central_line(mobileye_pb, path_length):
    ref_lane_x = []
    ref_lane_y = []

    rc0 = mobileye_pb.lka_768.position
    rc1 = mobileye_pb.lka_769.heading_angle
    rc2 = mobileye_pb.lka_768.curvature
    rc3 = mobileye_pb.lka_768.curvature_derivative
    # rrangex = mobileye_pb.lka_769.view_range

    lc0 = mobileye_pb.lka_766.position
    lc1 = mobileye_pb.lka_767.heading_angle
    lc2 = mobileye_pb.lka_766.curvature
    lc3 = mobileye_pb.lka_766.curvature_derivative
    # lrangex = mobileye_pb.lka_767.view_range
    offset = get_central_line_offset((rc0 + lc0) / 2.0)

    for y in range(int(path_length)):
        rx = rc3 * (y * y * y) + rc2 * (y * y) + rc1 * y + rc0
        lx = lc3 * (y * y * y) + lc2 * (y * y) + lc1 * y + lc0
        ref_lane_x.append((rx + lx) / 2.0 + offset)
        ref_lane_y.append(y)
    return ref_lane_x, ref_lane_y


def get_path(x, y, path_length):
    ind = int(math.floor((abs(x[0]) * 100.0) / 1) + 1)
    newx = [0]
    newy = [0]
    w = [1000]
    if len(y) - ind > 0:
        for i in range(len(y) - ind):
            newx.append(x[i + ind])
            newy.append(y[i + ind])
            w.append(w[-1] - 10)
    else:
        newx.append(x[-1])
        newy.append(y[-1])
        w.append(w[-1] - 10)
    coefs = poly.polyfit(newy, newx, 4, w=w)  # x = f(y)
    nx = poly.polyval(y, coefs)
    return nx, y


def get_speed():
    return CRUISE_SPEED


def localization_callback(localization_pb):
    speed_x = localization_pb.pose.linear_velocity.x
    speed_y = localization_pb.pose.linear_velocity.y
    acc_x = localization_pb.linear_acceleration.x
    acc_y = localization_pb.linear_acceleration.y


def chassis_callback(chassis_pb):
    global SPEED
    SPEED = chassis_pb.speed_mps


def euclidean_distance(point1, point2):
    sum = (point1[0] - point2[0]) * (point1[0] - point2[0])
    sum += (point1[1] - point2[1]) * (point1[1] - point2[1])
    return math.sqrt(sum)


def get_theta(point, point_base):
    # print point
    return math.atan2(point[1] - point_base[1],
                      point[0] - point_base[0]) - math.atan2(1, 0)


def mobileye_callback(mobileye_pb):
    global x, y, nx, ny, LAST_INIT_LAT
    start_timestamp = time.time()
    path_length = MINIMUM_PATH_LENGTH
    if path_length < SPEED * 2:
        path_length = math.ceil(SPEED * 2)
    x, y = get_central_line(mobileye_pb, path_length)
    nx, ny = x, y  # get_path(x, y, path_length)

    adc_trajectory = planning_pb2.ADCTrajectory()
    adc_trajectory.header.timestamp_sec = rospy.Time.now().to_sec()
    adc_trajectory.header.module_name = "planning"
    adc_trajectory.gear = chassis_pb2.Chassis.GEAR_DRIVE
    adc_trajectory.latency_stats.total_time_ms = \
        (time.time() - start_timestamp) * 1000
    s = 0
    relative_time = 0

    for i in range(len(nx)):
        traj_point = adc_trajectory.trajectory_point.add()
        traj_point.path_point.x = ny[i]
        traj_point.path_point.y = -nx[i]
        if i > 0:
            dist = euclidean_distance((nx[i], ny[i]), (nx[i - 1], ny[i - 1]))
            s += dist
            relative_time += dist / CRUISE_SPEED
        if (i + 1) >= len(nx):
            traj_point.path_point.theta = get_theta(
                (nx[-1], ny[-1]), (nx[0], ny[0]))
        else:
            traj_point.path_point.theta = get_theta(
                (nx[i + 1], ny[i + 1]), (nx[0], ny[0]))
        traj_point.path_point.s = s
        traj_point.v = CRUISE_SPEED
        traj_point.relative_time = relative_time

    planning_pub.publish(adc_trajectory)
    LAST_INIT_LAT = nx[0]
    f.write("duration: " + str(time.time() - start_timestamp) + "\n")


def add_listener():
    global planning_pub
    rospy.init_node(PUB_NODE_NAME, anonymous=True)
    rospy.Subscriber('/apollo/sensor/mobileye',
                     mobileye_pb2.Mobileye,
                     mobileye_callback)
    # rospy.Subscriber('/apollo/localization/pose',
    #                 localization_pb2.LocalizationEstimate,
    #                 localization_callback)
    rospy.Subscriber('/apollo/canbus/chassis',
                     chassis_pb2.Chassis,
                     chassis_callback)

    planning_pub = rospy.Publisher(
        PUB_TOPIC, planning_pb2.ADCTrajectory, queue_size=1)


def update(frame_number):
    line1.set_xdata(x)
    line1.set_ydata(y)
    line2.set_xdata(nx)
    line2.set_ydata(ny)


if __name__ == '__main__':

    DEBUG = False
    line1 = None
    line2 = None
    add_listener()

    if DEBUG:
        fig = plt.figure()
        ax = plt.subplot2grid((1, 1), (0, 0), rowspan=1, colspan=1)
        line1, = ax.plot([-10, 10, -10, 10], [-10, 150, 150, -10])
        line2, = ax.plot([0], [0])
        ani = animation.FuncAnimation(fig, update, interval=100)
        ax.axvline(x=0.0, alpha=0.3)
        ax.axhline(y=0.0, alpha=0.3)

        plt.show()
    else:
        rospy.spin()
