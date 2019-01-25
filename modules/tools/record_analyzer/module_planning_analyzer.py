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
from common.distribution_analyzer import DistributionAnalyzer
from common.error_code_analyzer import ErrorCodeAnalyzer
from common.error_msg_analyzer import ErrorMsgAnalyzer
from common.frechet_distance import frechet_distance
from modules.planning.proto import planning_pb2
from shapely.geometry import LineString, Point


class PlannigAnalyzer:
    """planning analyzer"""

    def __init__(self, is_simulation):
        """init"""
        self.module_latency = []
        self.trajectory_type_dist = {}
        self.estop_reason_dist = {}
        self.error_code_analyzer = ErrorCodeAnalyzer()
        self.error_msg_analyzer = ErrorMsgAnalyzer()
        self.last_adc_trajectory = None
        self.frechet_distance_list = []
        self.is_simulation = is_simulation
        self.hard_break_list = []
        self.total_cycle_num = 0

    def put(self, adc_trajectory):
        self.total_cycle_num += 1
        """put"""
        if not self.is_simulation:
            latency = adc_trajectory.latency_stats.total_time_ms
            self.module_latency.append(latency)

            self.error_code_analyzer.put(
                adc_trajectory.header.status.error_code)
            self.error_msg_analyzer.put(adc_trajectory.header.status.msg)

            traj_type = planning_pb2.ADCTrajectory.TrajectoryType.Name(
                adc_trajectory.trajectory_type)
            self.trajectory_type_dist[traj_type] = \
                self.trajectory_type_dist.get(traj_type, 0) + 1

            if adc_trajectory.estop.is_estop:
                self.estop_reason_dist[adc_trajectory.estop.reason] = \
                    self.estop_reason_dist.get(
                        adc_trajectory.estop.reason, 0) + 1

        if self.is_simulation:
            for point in adc_trajectory.trajectory_point:
                if point.a <= -2.0:
                    self.hard_break_list.append(point.a)

        if self.last_adc_trajectory is not None:
            current_path, last_path = self.find_common_path(adc_trajectory,
                                                            self.last_adc_trajectory)
            if len(current_path) == 0 or len(last_path) == 0:
                dist = 0
            else:
                dist = frechet_distance(current_path, last_path)
                self.frechet_distance_list.append(dist)

        self.last_adc_trajectory = adc_trajectory

    def find_common_path(self, current_adc_trajectory, last_adc_trajectory):
        current_path_points = current_adc_trajectory.trajectory_point
        last_path_points = last_adc_trajectory.trajectory_point
        current_path = []
        for point in current_path_points:
            current_path.append([point.path_point.x, point.path_point.y])
        last_path = []
        for point in last_path_points:
            last_path.append([point.path_point.x, point.path_point.y])
        if len(current_path) == 0 or len(last_path) == 0:
            return [], []

        current_ls = LineString(current_path)
        last_ls = LineString(last_path)
        current_start_point = Point(current_path[0])

        dist = last_ls.project(current_start_point)
        cut_lines = self.cut(last_ls, dist)
        if len(cut_lines) == 1:
            return [], []
        last_ls = cut_lines[1]
        dist = current_ls.project(Point(last_path[-1]))
        if dist <= current_ls.length:
            current_ls = self.cut(current_ls, dist)[0]
        else:
            dist = last_ls.project(Point(current_path[-1]))
            last_ls = self.cut(last_ls, dist)[0]
        return current_ls.coords, last_ls.coords

    def cut(self, line, distance):
        if distance <= 0.0 or distance >= line.length:
            return [LineString(line)]
        coords = list(line.coords)
        for i, p in enumerate(coords):
            pd = line.project(Point(p))
            if pd == distance:
                return [
                    LineString(coords[:i+1]),
                    LineString(coords[i:])]
            if pd > distance:
                cp = line.interpolate(distance)
                return [
                    LineString(coords[:i] + [(cp.x, cp.y)]),
                    LineString([(cp.x, cp.y)] + coords[i:])]

    def print_latency_statistics(self):
        """print_latency_statistics"""
        print "\n\n"
        print PrintColors.HEADER + "--- Planning Latency (ms) ---" + \
            PrintColors.ENDC
        StatisticalAnalyzer().print_statistical_results(self.module_latency)

        print PrintColors.HEADER + "--- Planning Trajectroy Type Distribution" \
                                   " ---" + PrintColors.ENDC
        DistributionAnalyzer().print_distribution_results(
            self.trajectory_type_dist)

        print PrintColors.HEADER + "--- Planning Estop Distribution" \
                                   " ---" + PrintColors.ENDC
        DistributionAnalyzer().print_distribution_results(
            self.estop_reason_dist)

        print PrintColors.HEADER + "--- Planning Error Code Distribution---" + \
            PrintColors.ENDC
        self.error_code_analyzer.print_results()
        print PrintColors.HEADER + "--- Planning Error Msg Distribution ---" + \
            PrintColors.ENDC
        self.error_msg_analyzer.print_results()

        print PrintColors.HEADER + "--- Planning Trajectory Frechet Distance (m) ---" + \
            PrintColors.ENDC
        StatisticalAnalyzer().print_statistical_results(self.frechet_distance_list)

    def print_simulation_results(self):
        results = {}
        results['frechet_dist'] = sum(self.frechet_distance_list) /\
            len(self.frechet_distance_list)
        results['hard_brake_cycle_num'] = len(self.hard_break_list)
        results['overall_score'] = 1- results['hard_brake_cycle_num'] /\
            float(self.total_cycle_num)
        if results['frechet_dist'] > 10:
            results['overall_score'] += 0.0
        else:
            results['overall_score'] += (1- results['frechet_dist'] / 10.0)
        results['overall_score'] /= 2.0
        

        print str(results)
