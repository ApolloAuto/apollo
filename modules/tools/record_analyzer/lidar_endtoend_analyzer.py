#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

from common.statistical_analyzer import StatisticalAnalyzer
from common.statistical_analyzer import PrintColors


class LidarEndToEndAnalyzer:
    """control analyzer"""

    def __init__(self):
        """init"""
        self.control_endtoend_latency = []
        self.control_unprocessed_lidar_timestamps = []

        self.planning_endtoend_latency = []
        self.planning_unprocessed_lidar_timestamps = []

    def put_control(self, control_cmd):
        """put control data"""
        if control_cmd.header.lidar_timestamp in \
                self.control_unprocessed_lidar_timestamps:
            ind = self.control_unprocessed_lidar_timestamps.index(
                control_cmd.header.lidar_timestamp)
            del (self.control_unprocessed_lidar_timestamps[ind])
            self.control_endtoend_latency.append(
                control_cmd.header.lidar_timestamp / 1.0e-9 -
                control_cmd.header.timestamp_sec)

    def put_planning(self, planning_cmd):
        """put control data"""
        if planning_cmd.header.lidar_timestamp in \
                self.planning_unprocessed_lidar_timestamps:
            ind = self.planning_unprocessed_lidar_timestamps.index(
                planning_cmd.header.lidar_timestamp)
            del (self.planning_unprocessed_lidar_timestamps[ind])
            self.planning_endtoend_latency.append(
                planning_cmd.header.lidar_timestamp * 1.0e-9 -
                planning_cmd.header.timestamp_sec)

    def put_lidar(self, point_cloud):
        """put lidar data"""
        self.control_unprocessed_lidar_timestamps.append(
            point_cloud.header.lidar_timestamp)
        self.planning_unprocessed_lidar_timestamps.append(
            point_cloud.header.lidar_timestamp)

    def print_endtoend_latency(self):
        """print_endtoend_latency"""
        print "\n\n"
        print PrintColors.HEADER + "* End to End (Control) Latency (ms)" + \
            PrintColors.ENDC
        analyzer = StatisticalAnalyzer()
        analyzer.print_statistical_results(self.control_endtoend_latency)

        print PrintColors.FAIL + "  - MISS # OF LIDAR: " + \
            str(len(self.control_unprocessed_lidar_timestamps)) + \
            PrintColors.ENDC

        print PrintColors.HEADER + "* End to End (Planning) Latency (ms)" + \
            PrintColors.ENDC
        analyzer = StatisticalAnalyzer()
        analyzer.print_statistical_results(self.planning_endtoend_latency)

        print PrintColors.FAIL + "  - MISS # OF LIDAR: " + \
            str(len(self.planning_unprocessed_lidar_timestamps)) + \
            PrintColors.ENDC
