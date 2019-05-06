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

import sys
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

        self.init_point_curvature = []
        self.init_point_dcurvature = []
        self.init_point_accel = []
        self.init_point_decel = []

        self.last_init_point_t = None
        self.last_init_point_a = None
        self.last_init_point = None

        self.centripetal_jerk_list = []
        self.centripetal_accel_list = []
        self.jerk_list = []

        self.latency_list = []

        # [2, 4) unit m/s^2
        self.ACCEL_M_LB = 2
        self.ACCEL_M_UB = 4
        self.accel_medium_cnt = 0

        # [4, ) unit m/s^2
        self.ACCEL_H_LB = 4
        self.accel_high_cnt = 0

        # [-4, -2)
        self.DECEL_M_LB = -4
        self.DECEL_M_UB = -2
        self.decel_medium_cnt = 0

        # [-4, )
        self.DECEL_H_UB = -4
        self.decel_high_cnt = 0

        # [1,2) (-2, -1]
        self.JERK_M_LB_P = 1
        self.JERK_M_UB_P = 2
        self.JERK_M_LB_N = -2
        self.JERK_M_UB_N = -1
        self.jerk_medium_cnt = 0

        # [2, inf) (-inf, -2]
        self.JERK_H_LB_P = 2
        self.JERK_H_UB_N = -2
        self.jerk_high_cnt = 0

        # [1, 2) [-2, -1)
        self.LAT_ACCEL_M_LB_P = 1
        self.LAT_ACCEL_M_UB_P = 2
        self.LAT_ACCEL_M_LB_N = -2
        self.LAT_ACCEL_M_UB_N = -1
        self.lat_accel_medium_cnt = 0

        # [2, inf)  [-inf,-2)
        self.LAT_ACCEL_H_LB_P = 2
        self.LAT_ACCEL_H_UB_N = -2
        self.lat_accel_high_cnt = 0

        # [0.5,1) [-1, -0.5)
        self.LAT_JERK_M_LB_P = 0.5
        self.LAT_JERK_M_UB_P = 1
        self.LAT_JERK_M_LB_N = -1
        self.LAT_JERK_M_UB_N = -0.5
        self.lat_jerk_medium_cnt = 0

        # [1, inf)  [-inf,-1)
        self.LAT_JERK_H_LB_P = 1
        self.LAT_JERK_H_UB_N = -1
        self.lat_jerk_high_cnt = 0

        self.bag_start_time_t = None
        self.print_acc = arguments.showacc

        self.rl_is_offroad_cnt = 0
        self.rl_minimum_boundary = sys.float_info.max
        self.rl_avg_kappa_list = []
        self.rl_avg_dkappa_list = []

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

        if self.is_sim:
            self.latency_list.append(adc_trajectory.latency_stats.total_time_ms)

            for ref_line_debug in adc_trajectory.debug.planning_data.reference_line:
                if ref_line_debug.HasField("is_offroad") and ref_line_debug.is_offroad:
                    self.rl_is_offroad_cnt += 1
                if ref_line_debug.HasField("minimum_boundary") and \
                    ref_line_debug.minimum_boundary < self.rl_minimum_boundary:
                    self.rl_minimum_boundary = ref_line_debug.minimum_boundary
                if ref_line_debug.HasField("average_kappa"):
                    self.rl_avg_kappa_list.append(abs(ref_line_debug.average_kappa))
                if ref_line_debug.HasField("average_dkappa"):
                    self.rl_avg_dkappa_list.append(abs(ref_line_debug.average_dkappa))

            if not adc_trajectory.debug.planning_data.HasField('init_point'):
                return

            init_point = adc_trajectory.debug.planning_data.init_point

            self.init_point_curvature.append(abs(init_point.path_point.kappa))
            self.init_point_dcurvature.append(abs(init_point.path_point.dkappa))

            t = adc_trajectory.header.timestamp_sec + init_point.relative_time

            if self.bag_start_time_t is None:
                self.bag_start_time_t = t

            accel = None
            jerk = None
            duration = 0
            if self.last_init_point_t is not None:
                duration = t - self.last_init_point_t
            if self.last_init_point is not None and duration > 0.03:
                accel = (init_point.v - self.last_init_point.v) / duration
                if self.print_acc and abs(accel) > 4:
                    print("---------------")
                    print(t - self.bag_start_time_t, "acc = ", accel)
                if accel > 0:
                    self.init_point_accel.append(accel)
                elif accel < 0:
                    self.init_point_decel.append(abs(accel))

                if self.DECEL_M_LB < accel <= self.DECEL_M_UB:
                    self.decel_medium_cnt += 1
                if accel <= self.DECEL_H_UB:
                    self.decel_high_cnt += 1

                if self.ACCEL_M_LB <= accel < self.ACCEL_M_UB:
                    self.accel_medium_cnt += 1
                if self.ACCEL_H_LB <= accel:
                    self.accel_high_cnt += 1

                if self.last_init_point_a is not None:
                    if duration <= 0:
                        jerk = 0
                    else:
                        jerk = (accel - self.last_init_point_a) / duration
                    self.jerk_list.append(abs(jerk))

                    if self.JERK_M_LB_P <= jerk < self.JERK_M_UB_P or \
                        self.JERK_M_LB_N < jerk <= self.JERK_M_UB_N:
                        self.jerk_medium_cnt += 1
                    if jerk >= self.JERK_H_LB_P or jerk <= self.JERK_H_UB_N:
                        self.jerk_high_cnt += 1

                # centripetal_jerk
                centripetal_jerk = 2 * init_point.v * init_point.a \
                    * init_point.path_point.kappa + init_point.v \
                        * init_point.v * init_point.path_point.dkappa
                self.centripetal_jerk_list.append(abs(centripetal_jerk))

                if self.LAT_JERK_M_LB_P <= centripetal_jerk < self.LAT_JERK_M_UB_P \
                    or self.LAT_JERK_M_LB_N < centripetal_jerk <= self.LAT_JERK_M_UB_N:
                    self.lat_jerk_medium_cnt += 1
                if centripetal_jerk >= self.LAT_JERK_H_LB_P \
                    or centripetal_jerk <= self.LAT_JERK_H_UB_N:
                    self.lat_jerk_high_cnt += 1

                # centripetal_accel
                centripetal_accel = init_point.v * init_point.v \
                    * init_point.path_point.kappa
                self.centripetal_accel_list.append(abs(centripetal_accel))

                if self.LAT_ACCEL_M_LB_P <= centripetal_accel < self.LAT_ACCEL_M_UB_P \
                    or self.LAT_ACCEL_M_LB_N < centripetal_accel <= self.LAT_ACCEL_M_UB_N:
                    self.lat_accel_medium_cnt += 1
                if centripetal_accel >= self.LAT_ACCEL_H_LB_P \
                    or centripetal_accel <= self.LAT_ACCEL_H_UB_N:
                    self.lat_accel_high_cnt += 1

            self.last_init_point_t = t
            self.last_init_point = init_point
            self.last_init_point_a = accel

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

    def print_sim_results(self):
        """
        dreamland metrics for planning v2
        """
        v2_results = {}

        # acceleration
        v2_results["accel"] = {}
        if len(self.init_point_accel) > 0:
            v2_results["accel"]["max"] = max(self.init_point_accel)
            v2_results["accel"]["avg"] = np.average(self.init_point_accel)
        else:
            v2_results["accel"]["max"] = 0.0
            v2_results["accel"]["avg"] = 0.0
        v2_results["accel"]["medium_cnt"] = self.accel_medium_cnt
        v2_results["accel"]["high_cnt"] = self.accel_high_cnt

        # deceleration
        v2_results["decel"] = {}
        if len(self.init_point_decel) > 0:
            v2_results["decel"]["max"] = max(self.init_point_decel)
            v2_results["decel"]["avg"] = np.average(self.init_point_decel)
        else:
            v2_results["decel"]["max"] = 0.0
            v2_results["decel"]["avg"] = 0.0
        v2_results["decel"]["medium_cnt"] = self.decel_medium_cnt
        v2_results["decel"]["high_cnt"] = self.decel_high_cnt

        # jerk
        v2_results["jerk"] = {}
        if len(self.jerk_list) > 0:
            v2_results["jerk"]["max"] = max(self.jerk_list, key=abs)
            jerk_avg = np.average(np.absolute(self.jerk_list))
            v2_results["jerk"]["avg"] = jerk_avg
        else:
            v2_results["jerk"]["max"] = 0
            v2_results["jerk"]["avg"] = 0
        v2_results["jerk"]["medium_cnt"] = self.jerk_medium_cnt
        v2_results["jerk"]["high_cnt"] = self.jerk_high_cnt

        # centripetal_jerk
        v2_results["lat_jerk"] = {}
        if len(self.centripetal_jerk_list) > 0:
            v2_results["lat_jerk"]["max"] = max(self.centripetal_jerk_list, key=abs)
            jerk_avg = np.average(np.absolute(self.centripetal_jerk_list))
            v2_results["lat_jerk"]["avg"] = jerk_avg
        else:
            v2_results["lat_jerk"]["max"] = 0
            v2_results["lat_jerk"]["avg"] = 0
        v2_results["lat_jerk"]["medium_cnt"] = self.lat_jerk_medium_cnt
        v2_results["lat_jerk"]["high_cnt"] = self.lat_jerk_high_cnt

        # centripetal_accel
        v2_results["lat_accel"] = {}
        if len(self.centripetal_accel_list) > 0:
            v2_results["lat_accel"]["max"] = max(self.centripetal_accel_list, key=abs)
            accel_avg = np.average(np.absolute(self.centripetal_accel_list))
            v2_results["lat_accel"]["avg"] = accel_avg
        else:
            v2_results["lat_accel"]["max"] = 0
            v2_results["lat_accel"]["avg"] = 0
        v2_results["lat_accel"]["medium_cnt"] = self.lat_accel_medium_cnt
        v2_results["lat_accel"]["high_cnt"] = self.lat_accel_high_cnt

        # latency
        if len(self.latency_list) > 0:
            v2_results["planning_latency"] = {
                "max" : max(self.latency_list),
                "min" : min(self.latency_list),
                "avg" : np.average(self.latency_list)
            }

        # reference line
        average_kappa = 0
        if len(self.rl_avg_kappa_list) > 0:
            average_kappa = np.average(self.rl_avg_kappa_list)
        average_dkappa = 0
        if len(self.rl_avg_dkappa_list) > 0:
            average_dkappa = np.average(self.rl_avg_dkappa_list)
        if self.rl_minimum_boundary > 999:
            self.rl_minimum_boundary = 0
        v2_results["reference_line"] = {
            "is_offroad" : self.rl_is_offroad_cnt,
            "minimum_boundary" : self.rl_minimum_boundary,
            "average_kappa" : average_kappa,
            "average_dkappa" : average_dkappa
        }

        # output final reuslts
        print json.dumps(v2_results)

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
