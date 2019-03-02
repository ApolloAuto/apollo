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

import json
import numpy as np
from shapely.geometry import LineString, Point
from modules.planning.proto import planning_pb2
from common.statistical_analyzer import StatisticalAnalyzer
from common.statistical_analyzer import PrintColors
from common.distribution_analyzer import DistributionAnalyzer
from common.error_code_analyzer import ErrorCodeAnalyzer
from common.error_msg_analyzer import ErrorMsgAnalyzer
from common.frechet_distance import frechet_distance


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
        self.init_point_curvature = []
        self.init_point_dcurvature = []

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

            if adc_trajectory.debug.planning_data.HasField('init_point'):
                self.init_point_curvature.append(
                    abs(adc_trajectory.debug.planning_data.init_point.path_point.kappa))
                self.init_point_dcurvature.append(
                    abs(adc_trajectory.debug.planning_data.init_point.path_point.dkappa))

        # TODO(yifei) temporarily disable frechet distance
        #if self.last_adc_trajectory is not None and self.is_simulation:
        #    current_path, last_path = self.find_common_path(adc_trajectory,
        #                                                    self.last_adc_trajectory)
        #    if len(current_path) == 0 or len(last_path) == 0:
        #        dist = 0
        #    else:
        #        dist = frechet_distance(current_path, last_path)
        #        if dist is not None:
        #            self.frechet_distance_list.append(dist)

        self.last_adc_trajectory = adc_trajectory

    def find_common_path(self, current_adc_trajectory, last_adc_trajectory):
        current_path_points = current_adc_trajectory.trajectory_point
        last_path_points = last_adc_trajectory.trajectory_point

        current_path = []
        for point in current_path_points:
            current_path.append([point.path_point.x, point.path_point.y])
            if point.path_point.s > 5.0:
                break
        last_path = []
        for point in last_path_points:
            last_path.append([point.path_point.x, point.path_point.y])
            if point.path_point.s > 5.0:
                break

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
        # TODO(yifei) temporarily disable frechet distance
        # results['frechet_dist'] = sum(self.frechet_distance_list) /\
        #    len(self.frechet_distance_list)

        results['hard_brake_cycle_num'] = len(self.hard_break_list)

        curvature_99pctl = np.percentile(self.init_point_curvature, 99)
        results['curvature_99pctl'] = curvature_99pctl
        curvature_avg = np.average(self.init_point_curvature)
        results['curvature_avg'] = curvature_avg

        dcurvature_99pctl = np.percentile(self.init_point_dcurvature, 99)
        results['dcurvature_99pctl'] = dcurvature_99pctl
        dcurvature_avg = np.average(self.init_point_dcurvature)
        results['dcurvature_avg'] = dcurvature_avg

        results['overall_score'] = 1 - results['hard_brake_cycle_num'] /\
            float(self.total_cycle_num)
        # TODO(yifei) temporarily disable frechet distance
        #if results['frechet_dist'] > 10:
        #    results['overall_score'] += 0.0
        #else:
        #    results['overall_score'] += (1 - results['frechet_dist'] / 10.0)
        #results['overall_score'] /= 2.0

        print json.dumps(results)

    def plot_path(self, plt, adc_trajectory):
        path_coords = self.trim_path_by_distance(adc_trajectory, 5.0)
        x = []
        y = []
        for point in path_coords:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, 'r-', alpha=0.5)

    def plot_refpath(self, plt, adc_trajectory):
        for path in adc_trajectory.debug.planning_data.path:
            if path.name != 'planning_reference_line':
                continue
            path_coords = self.trim_path_by_distance(adc_trajectory, 5.0)

            ref_path_coord = []
            for point in path.path_point:
                ref_path_coord.append([point.x, point.y])
            ref_path = LineString(ref_path_coord)

            start_point = Point(path_coords[0])
            dist = ref_path.project(start_point)
            ref_path = self.cut(ref_path, dist)[1]

            end_point = Point(path_coords[-1])
            dist = ref_path.project(end_point)
            ref_path = self.cut(ref_path, dist)[0]

            x = []
            y = []
            for point in ref_path.coords:
                x.append(point[0])
                y.append(point[1])

            plt.plot(x, y, 'b--', alpha=0.5)

    def trim_path_by_distance(self, adc_trajectory, s):
        path_coords = []
        path_points = adc_trajectory.trajectory_point
        for point in path_points:
            if point.path_point.s <= s:
                path_coords.append([point.path_point.x, point.path_point.y])
        return path_coords
