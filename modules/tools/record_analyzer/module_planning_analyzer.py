#!/usr/bin/env python3

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
import sys

import numpy as np

from modules.tools.record_analyzer.common.distribution_analyzer import DistributionAnalyzer
from modules.tools.record_analyzer.common.error_code_analyzer import ErrorCodeAnalyzer
from modules.tools.record_analyzer.common.error_msg_analyzer import ErrorMsgAnalyzer
from modules.tools.record_analyzer.common.frechet_distance import frechet_distance
from modules.tools.record_analyzer.common.statistical_analyzer import PrintColors
from modules.tools.record_analyzer.common.statistical_analyzer import StatisticalAnalyzer
from modules.tools.record_analyzer.metrics.curvature import Curvature
from modules.tools.record_analyzer.metrics.frame_count import FrameCount
from modules.tools.record_analyzer.metrics.latency import Latency
from modules.tools.record_analyzer.metrics.lat_acceleration import LatAcceleration
from modules.tools.record_analyzer.metrics.lon_acceleration import LonAcceleration
from modules.tools.record_analyzer.metrics.reference_line import ReferenceLine
from modules.common_msgs.planning_msgs import planning_pb2
from shapely.geometry import LineString, Point


class PlannigAnalyzer:
    """planning analyzer"""

    def __init__(self, arguments):
        """init"""
        self.module_latency = []
        self.trajectory_type_dist = {}
        self.estop_reason_dist = {}
        self.error_code_analyzer = ErrorCodeAnalyzer()
        self.error_msg_analyzer = ErrorMsgAnalyzer()
        self.last_adc_trajectory = None
        self.frechet_distance_list = []
        self.is_sim = arguments.simulation
        self.hard_break_list = []
        self.total_cycle_num = 0

        self.curvature_analyzer = Curvature()
        self.frame_count_analyzer = FrameCount()
        self.lat_acceleration_analyzer = LatAcceleration()
        self.lon_acceleration_analyzer = LonAcceleration()
        self.latency_analyzer = Latency()
        self.reference_line = ReferenceLine()

        self.bag_start_time_t = None
        self.print_acc = arguments.showacc

    def put(self, adc_trajectory):
        self.total_cycle_num += 1
        if not self.is_sim:
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

        else:
            self.curvature_analyzer.put(adc_trajectory)
            self.frame_count_analyzer.put(adc_trajectory)
            self.lat_acceleration_analyzer.put(adc_trajectory)
            self.lon_acceleration_analyzer.put(adc_trajectory)
            self.latency_analyzer.put(adc_trajectory)
            self.reference_line.put(adc_trajectory)

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
        print("\n\n")
        print(PrintColors.HEADER + "--- Planning Latency (ms) ---" +
              PrintColors.ENDC)
        StatisticalAnalyzer().print_statistical_results(self.module_latency)

        print(PrintColors.HEADER + "--- Planning Trajectroy Type Distribution"
                                   " ---" + PrintColors.ENDC)
        DistributionAnalyzer().print_distribution_results(
            self.trajectory_type_dist)

        print(PrintColors.HEADER + "--- Planning Estop Distribution"
                                   " ---" + PrintColors.ENDC)
        DistributionAnalyzer().print_distribution_results(
            self.estop_reason_dist)

        print(PrintColors.HEADER + "--- Planning Error Code Distribution---" +
              PrintColors.ENDC)
        self.error_code_analyzer.print_results()
        print(PrintColors.HEADER + "--- Planning Error Msg Distribution ---" +
              PrintColors.ENDC)
        self.error_msg_analyzer.print_results()

        print(PrintColors.HEADER + "--- Planning Trajectory Frechet Distance (m) ---" +
              PrintColors.ENDC)
        StatisticalAnalyzer().print_statistical_results(self.frechet_distance_list)

    def print_sim_results(self):
        """
        dreamland metrics for planning v2
        """
        v2_results = {}

        # acceleration
        v2_results["accel"] = self.lon_acceleration_analyzer.get_acceleration()

        # deceleration
        v2_results["decel"] = self.lon_acceleration_analyzer.get_deceleration()

        # jerk
        v2_results["acc_jerk"] = self.lon_acceleration_analyzer.get_acc_jerk()
        v2_results["dec_jerk"] = self.lon_acceleration_analyzer.get_dec_jerk()

        # centripetal_jerk
        v2_results["lat_jerk"] = self.lat_acceleration_analyzer.get_jerk()

        # centripetal_accel
        v2_results["lat_accel"] = self.lat_acceleration_analyzer.get_acceleration()

        # frame_count
        v2_results["frame_count"] = self.frame_count_analyzer.get()

        # latency
        v2_results["planning_latency"] = self.latency_analyzer.get()

        # reference line
        v2_results["reference_line"] = self.reference_line.get()

        # output final reuslts
        print(json.dumps(v2_results))

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
