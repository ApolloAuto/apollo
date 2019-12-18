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

import matplotlib.pyplot as plt

import common.proto_utils as proto_utils
import mkz_polygon
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.localization.proto.localization_pb2 import LocalizationEstimate


def plot_trajectory(planning_pb, ax):
    points_x = []
    points_y = []
    points_t = []
    base_time_sec = planning_pb.header.timestamp_sec
    for trajectory_point in planning_pb.adc_trajectory_point:
        points_x.append(trajectory_point.x)
        points_y.append(trajectory_point.y)
        points_t.append(base_time_sec + trajectory_point.relative_time)
    ax.plot(points_x, points_y, "r.")


def find_closest_t(points_t, current_t):
    if len(points_t) == 0:
        return -1
    if len(points_t) == 1:
        return points_t[0]
    if len(points_t) == 2:
        if abs(points_t[0] - current_t) < abs(points_t[1] - current_t):
            return points_t[0]
        else:
            return points_t[1]
    if points_t[len(points_t) / 2] > current_t:
        return find_closest_t(points_t[0:len(points_t) / 2], current_t)
    elif points_t[len(points_t) / 2] < current_t:
        return find_closest_t(points_t[len(points_t) / 2 + 1:], current_t)
    else:
        return current_t


def find_closest_traj_point(planning_pb, current_t):
    points_x = []
    points_y = []
    points_t = []
    base_time_sec = planning_pb.header.timestamp_sec
    for trajectory_point in planning_pb.adc_trajectory_point:
        points_x.append(trajectory_point.x)
        points_y.append(trajectory_point.y)
        points_t.append(base_time_sec + trajectory_point.relative_time)

    matched_t = find_closest_t(points_t, current_t)
    idx = points_t.index(matched_t)
    return planning_pb.adc_trajectory_point[idx]


def plot_traj_point(planning_pb, traj_point, ax):
    matched_t = planning_pb.header.timestamp_sec \
        + traj_point.relative_time
    ax.plot([traj_point.x], [traj_point.y], "bs")
    content = "t = " + str(matched_t) + "\n"
    content += "speed = " + str(traj_point.speed) + "\n"
    content += "acc = " + str(traj_point.acceleration_s)
    lxy = [-80, -80]
    ax.annotate(
        content,
        xy=(traj_point.x, traj_point.y),
        xytext=lxy,
        textcoords='offset points',
        ha='right',
        va='top',
        bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.3),
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
        alpha=0.8)


def plot_vehicle(localization_pb, ax):
    loc_x = [localization_pb.pose.position.x]
    loc_y = [localization_pb.pose.position.y]
    current_t = localization_pb.header.timestamp_sec
    ax.plot(loc_x, loc_y, "bo")
    position = []
    position.append(localization_pb.pose.position.x)
    position.append(localization_pb.pose.position.y)
    position.append(localization_pb.pose.position.z)

    mkz_polygon.plot(position, localization_pb.pose.heading, ax)
    content = "t = " + str(current_t) + "\n"
    content += "speed @y = " + \
        str(localization_pb.pose.linear_velocity.y) + "\n"
    content += "acc @y = " + \
        str(localization_pb.pose.linear_acceleration_vrf.y)
    lxy = [-80, 80]
    ax.annotate(
        content,
        xy=(loc_x[0], loc_y[0]),
        xytext=lxy,
        textcoords='offset points',
        ha='left',
        va='top',
        bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.3),
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
        alpha=0.8)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: %s <planning.pb.txt> <localization.pb.txt>" % sys.argv[0])
        sys.exit(0)

    planning_pb_file = sys.argv[1]
    localization_pb_file = sys.argv[2]
    planning_pb = proto_utils.get_pb_from_text_file(
        planning_pb_file, ADCTrajectory())
    localization_pb = proto_utils.get_pb_from_text_file(
        localization_pb_file, LocalizationEstimate())

    plot_trajectory(planning_pb, plt)
    plot_vehicle(localization_pb, plt)

    current_t = localization_pb.header.timestamp_sec
    trajectory_point = find_closest_traj_point(planning_pb, current_t)
    plot_traj_point(planning_pb, trajectory_point, plt)

    plt.axis('equal')
    plt.show()
