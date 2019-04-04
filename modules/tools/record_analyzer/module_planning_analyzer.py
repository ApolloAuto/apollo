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

    def __init__(self, is_simulation, is_sim):
        """init"""
        self.module_latency = []
        self.trajectory_type_dist = {}
        self.estop_reason_dist = {}
        self.error_code_analyzer = ErrorCodeAnalyzer()
        self.error_msg_analyzer = ErrorMsgAnalyzer()
        self.last_adc_trajectory = None
        self.frechet_distance_list = []
        self.is_simulation = is_simulation
        self.is_sim = is_sim
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

        self.breaking_2_3_cnt = 0
        self.breaking_3_5_cnt = 0
        self.breaking_5_cnt = 0
        self.throttle_2_3_cnt = 0
        self.throttle_3_5_cnt = 0
        self.throttle_5_cnt = 0

    def put(self, adc_trajectory):
        self.total_cycle_num += 1
        if not self.is_simulation and not self.is_sim:
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

        if self.is_simulation or self.is_sim:
            if not adc_trajectory.debug.planning_data.HasField('init_point'):
                return

            init_point = adc_trajectory.debug.planning_data.init_point

            self.init_point_curvature.append(abs(init_point.path_point.kappa))
            self.init_point_dcurvature.append(abs(init_point.path_point.dkappa))

            t = adc_trajectory.header.timestamp_sec + init_point.relative_time

            accel = None
            jerk = None
            if self.last_init_point is not None:
                duration = t - self.last_init_point_t
                if duration <= 0:
                    accel = 0
                else:
                    accel = (init_point.v - self.last_init_point.v) / duration
                if accel > 0:
                    self.init_point_accel.append(accel)
                if accel < 0:
                    self.init_point_decel.append(abs(accel))

                if -3 < accel <= -2:
                    self.breaking_2_3_cnt += 1
                if -5 < accel <= -3:
                    self.breaking_3_5_cnt += 1
                if accel <= -5:
                    self.breaking_5_cnt += 1

                if 2 <= accel < 3:
                    self.throttle_2_3_cnt += 1
                if 3 <= accel < 5:
                    self.throttle_3_5_cnt += 1
                if 5 <= accel :
                    self.throttle_5_cnt += 1

                if self.last_init_point_a is not None:
                    if duration <= 0:
                        jerk = 0
                    else:
                        jerk = (accel - self.last_init_point_a) / duration
                        # print accel, self.last_init_point_a, duration, jerk
                    self.jerk_list.append(jerk)

                centripetal_jerk = 2 * init_point.v * init_point.a \
                    * init_point.path_point.kappa + init_point.v \
                        * init_point.v * init_point.path_point.dkappa
                self.centripetal_jerk_list.append(centripetal_jerk)

                centripetal_accel = init_point.v * init_point.v \
                    * init_point.path_point.kappa
                self.centripetal_accel_list.append(centripetal_accel)

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

    def print_simulation_results(self):
        results = {}

        results['decel_2_3'] = self.breaking_2_3_cnt
        results['decel_3_5'] = self.breaking_3_5_cnt
        results['decel_5_'] = self.breaking_5_cnt

        results['accel_2_3'] = self.throttle_2_3_cnt
        results['accel_3_5'] = self.throttle_3_5_cnt
        results['accel_5_'] = self.throttle_5_cnt

        if len(self.init_point_curvature) > 0:
            results['curvature_max'] = max(self.init_point_curvature, key=abs)
            curvature_avg = np.average(np.absolute(self.init_point_curvature))
            results['curvature_avg'] = curvature_avg
        else:
            results['curvature_max'] = None
            results['curvature_avg'] = None

        if len(self.init_point_dcurvature) > 0:
            results['dcurvature_max'] = max(self.init_point_dcurvature, key=abs)
            dcurvature_avg = np.average(np.absolute(self.init_point_dcurvature))
            results['dcurvature_avg'] = dcurvature_avg
        else:
            results['dcurvature_max'] = None
            results['dcurvature_avg'] = None

        if len(self.init_point_accel) > 0:
            results["accel_max"] = max(self.init_point_accel)
            results["accel_avg"] = np.average(self.init_point_accel)
        else:
            results["accel_max"] = 0.0
            results["accel_avg"] = 0.0

        if len(self.init_point_decel) > 0:
            results["decel_max"] = max(self.init_point_decel)
            results["decel_avg"] = np.average(self.init_point_decel)
        else:
            results["decel_max"] = 0.0
            results["decel_avg"] = 0.0

        if len(self.jerk_list) > 0:
            results["jerk_max"] = max(self.jerk_list, key=abs)
            jerk_avg = np.average(np.absolute(self.jerk_list))
            results["jerk_avg"] = jerk_avg
        else:
            results["jerk_max"] = 0
            results["jerk_avg"] = 0

        if len(self.centripetal_jerk_list) > 0:
            results["centripetal_jerk_max"] = max(self.centripetal_jerk_list, key=abs)
            jerk_avg = np.average(np.absolute(self.centripetal_jerk_list))
            results["centripetal_jerk_avg"] = jerk_avg
        else:
            results["centripetal_jerk_max"] = 0
            results["centripetal_jerk_avg"] = 0

        if len(self.centripetal_accel_list) > 0:
            results["centripetal_accel_max"] = max(self.centripetal_accel_list, key=abs)
            accel_avg = np.average(np.absolute(self.centripetal_accel_list))
            results["centripetal_accel_avg"] = accel_avg
        else:
            results["centripetal_accel_max"] = 0
            results["centripetal_accel_avg"] = 0

        print json.dumps(results)

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
        v2_results["accel"]['2 to 3 cnt'] = self.throttle_2_3_cnt
        v2_results["accel"]['3 to 5 cnt'] = self.throttle_3_5_cnt
        v2_results["accel"]['5 and up cnt'] = self.throttle_5_cnt

        # deceleration
        v2_results["decel"] = {}
        if len(self.init_point_decel) > 0:
            v2_results["decel"]["max"] = max(self.init_point_decel)
            v2_results["decel"]["avg"] = np.average(self.init_point_decel)
        else:
            v2_results["decel"]["max"] = 0.0
            v2_results["decel"]["avg"] = 0.0
        v2_results["decel"]['2 to 3 cnt'] = self.breaking_2_3_cnt
        v2_results["decel"]['3 to 5 cnt'] = self.breaking_3_5_cnt
        v2_results["decel"]['5 and up cnt'] = self.breaking_5_cnt

        # jerk
        v2_results["jerk"] = {}
        if len(self.jerk_list) > 0:
            print self.jerk_list
            v2_results["jerk"]["max"] = max(self.jerk_list, key=abs)
            jerk_avg = np.average(np.absolute(self.jerk_list))
            v2_results["jerk"]["avg"] = jerk_avg
        else:
            v2_results["jerk"]["max"] = 0
            v2_results["jerk"]["avg"] = 0

        # centripetal_jerk
        v2_results["centripetal_jerk"] = {}
        if len(self.centripetal_jerk_list) > 0:
            v2_results["centripetal_jerk"]["max"] = max(self.centripetal_jerk_list, key=abs)
            jerk_avg = np.average(np.absolute(self.centripetal_jerk_list))
            v2_results["centripetal_jerk"]["avg"] = jerk_avg
        else:
            v2_results["centripetal_jerk"]["max"] = 0
            v2_results["centripetal_jerk"]["avg"] = 0

        # centripetal_accel
        v2_results["centripetal_accel"] = {}
        if len(self.centripetal_accel_list) > 0:
            v2_results["centripetal_accel"]["max"] = max(self.centripetal_accel_list, key=abs)
            accel_avg = np.average(np.absolute(self.centripetal_accel_list))
            v2_results["centripetal_accel"]["avg"] = accel_avg
        else:
            v2_results["centripetal_accel"]["max"] = 0
            v2_results["centripetal_accel"]["avg"] = 0

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
