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
import math
from numpy.polynomial.polynomial import polyval
from modules.common_msgs.planning_msgs import planning_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.basic_msgs import drive_state_pb2
from cyber.python.cyber_py3 import cyber_time


def euclidean_distance(point1, point2):
    sum = (point1[0] - point2[0]) * (point1[0] - point2[0])
    sum += (point1[1] - point2[1]) * (point1[1] - point2[1])
    return math.sqrt(sum)


def get_theta(point, point_base):
    # print point
    return math.atan2(1, 0) - math.atan2(point[0] - point_base[0],
                                         point[1] - point_base[1])


class TrajectoryGenerator:
    def __init__(self):
        self.mobileye_pb = None

    def generate(self, path, final_path_length, speed,
                 start_timestamp):
        path_x, path_y = path.get_xy()
        adc_trajectory = planning_pb2.ADCTrajectory()
        adc_trajectory.header.timestamp_sec = cyber_time.Time.now().to_sec()
        adc_trajectory.header.module_name = "planning"
        adc_trajectory.gear = chassis_pb2.Chassis.GEAR_DRIVE
        adc_trajectory.latency_stats.total_time_ms = \
            (cyber_time.Time.now().to_sec() - start_timestamp) * 1000
        s = 0
        relative_time = 0
        adc_trajectory.engage_advice.advice \
            = drive_state_pb2.EngageAdvice.READY_TO_ENGAGE

        for x in range(int(final_path_length - 1)):
            y = path_y[x]

            traj_point = adc_trajectory.trajectory_point.add()
            traj_point.path_point.x = x
            traj_point.path_point.y = y
            if x > 0:
                dist = euclidean_distance((x, y), (x - 1, path_y[x - 1]))
                s += dist
                relative_time += dist / speed

            traj_point.path_point.theta = get_theta(
                (x + 1, path_y[x + 1]), (0, path_y[0]))
            traj_point.path_point.s = s
            traj_point.v = speed
            traj_point.relative_time = relative_time
        return adc_trajectory
