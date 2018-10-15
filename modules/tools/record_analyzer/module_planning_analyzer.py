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

from statistical_analyzer import StatisticalAnalyzer
from statistical_analyzer import PrintColors
from distribution_analyzer import DistributionAnalyzer
from error_code_analyzer import ErrorCodeAnalyzer
from error_msg_analyzer import ErrorMsgAnalyzer
from modules.planning.proto import planning_pb2


class PlannigAnalyzer:
    """planning analyzer"""

    def __init__(self):
        """init"""
        self.module_latency = []
        self.trajectory_type_dist = {}
        self.error_code_analyzer = ErrorCodeAnalyzer()
        self.error_msg_analyzer = ErrorMsgAnalyzer()

    def put(self, adc_trajectory):
        """put"""
        latency = adc_trajectory.latency_stats.total_time_ms
        self.module_latency.append(latency)

        self.error_code_analyzer.put(adc_trajectory.header.status.error_code)
        self.error_msg_analyzer.put(adc_trajectory.header.status.msg)

        traj_type = planning_pb2.ADCTrajectory.TrajectoryType.Name(
            adc_trajectory.trajectory_type)
        self.trajectory_type_dist[traj_type] = \
            self.trajectory_type_dist.get(traj_type, 0) + 1

    def print_latency_statistics(self):
        """print_latency_statistics"""
        print "\n\n"
        print PrintColors.HEADER + "--- Planning Latency (ms) ---" + \
              PrintColors.ENDC
        StatisticalAnalyzer().print_statistical_results(self.module_latency)

        print PrintColors.HEADER + "--- Planning Trajectroy Type Distribution" \
                                   " (ms) ---" + PrintColors.ENDC
        DistributionAnalyzer().print_distribution_results(
            self.trajectory_type_dist)

        print PrintColors.HEADER + "--- Planning Error Code Distribution---" + \
              PrintColors.ENDC
        self.error_code_analyzer.print_results()
        print PrintColors.HEADER + "--- Planning Error Msg Distribution ---" + \
              PrintColors.ENDC
        self.error_msg_analyzer.print_results()
