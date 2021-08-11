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

import threading

import numpy as np

from modules.planning.proto import planning_internal_pb2


class Planning:
    def __init__(self, planning_pb=None):
        self.data_lock = threading.Lock()
        self.init_point_lock = threading.Lock()
        self.planning_pb = planning_pb
        self.path_data_lock = threading.Lock()
        self.path_data_x = {}
        self.path_data_y = {}
        self.speed_data_lock = threading.Lock()
        self.speed_data_time = {}
        self.speed_data_val = {}

        self.traj_data_lock = threading.Lock()
        self.traj_speed_history_len = 30
        self.traj_speed_t_history = []
        self.traj_speed_v_history = []
        self.traj_acc_history_len = 30
        self.traj_acc_t_history = []
        self.traj_acc_a_history = []
        self.traj_path_history_len = 30
        self.traj_path_x_history = []
        self.traj_path_y_history = []

        self.st_data_lock = threading.Lock()
        self.st_curve_s = {}
        self.st_curve_t = {}
        self.st_curve_v = {}
        self.st_data_boundary_s = {}
        self.st_data_boundary_t = {}
        self.st_data_boundary_type = {}
        self.st_speed_limit_s = {}
        self.st_speed_limit_v = {}

        self.st_speed_constraint_s = {}
        self.st_speed_constraint_lower = {}
        self.st_speed_constraint_upper = {}

        self.sl_data_lock = threading.Lock()
        self.sl_sampled_s = []
        self.sl_static_obstacle_lower_boundary = []
        self.sl_static_obstacle_upper_boundary = []
        self.sl_dynamic_obstacle_lower_boundary = []
        self.sl_dynamic_obstacle_upper_boundary = []
        self.sl_map_lower_boundary = []
        self.sl_map_upper_boundary = []
        self.sl_path_s = []
        self.sl_path_l = []
        self.sl_aggregated_boundary_low_l = []
        self.sl_aggregated_boundary_high_l = []
        self.sl_aggregated_boundary_s = []

        self.kernel_cruise_t = {}
        self.kernel_cruise_s = {}
        self.kernel_follow_t = {}
        self.kernel_follow_s = {}

        self.init_point_x = []
        self.init_point_y = []

    def update_planning_pb(self, planning_pb):
        self.planning_pb = planning_pb

    def compute_init_point(self):
        self.init_point_lock.acquire()
        init_point = self.planning_pb.debug.planning_data.init_point
        self.init_point_x = [init_point.path_point.x]
        self.init_point_y = [init_point.path_point.y]
        self.init_point_lock.release()

    def compute_sl_data(self):
        sl_sampled_s = []
        sl_map_lower_boundary = []
        sl_map_upper_boundary = []
        sl_static_obstacle_lower_boundary = []
        sl_static_obstacle_upper_boundary = []
        sl_dynamic_obstacle_lower_boundary = []
        sl_dynamic_obstacle_upper_boundary = []
        sl_path_s = []
        sl_path_l = []
        sl_aggregated_boundary_low_l = []
        sl_aggregated_boundary_high_l = []
        sl_aggregated_boundary_s = []

        for sl_frame in self.planning_pb.debug.planning_data.sl_frame:
            for s in sl_frame.sampled_s:
                sl_sampled_s.append(s)
            for l in sl_frame.map_lower_bound:
                if (l > 10 or l < -10):
                    sl_map_lower_boundary.append(100 * l // abs(l))
                else:
                    sl_map_lower_boundary.append(l)
            for l in sl_frame.map_upper_bound:
                if (l > 10 or l < -10):
                    sl_map_upper_boundary.append(100 * l // abs(l))
                else:
                    sl_map_upper_boundary.append(l)
            for l in sl_frame.static_obstacle_lower_bound:
                sl_static_obstacle_lower_boundary.append(l)
            for l in sl_frame.static_obstacle_upper_bound:
                sl_static_obstacle_upper_boundary.append(l)
            for l in sl_frame.dynamic_obstacle_lower_bound:
                sl_dynamic_obstacle_lower_boundary.append(l)
            for l in sl_frame.dynamic_obstacle_upper_bound:
                sl_dynamic_obstacle_upper_boundary.append(l)
            for slpoint in sl_frame.sl_path:
                sl_path_s.append(slpoint.s)
                sl_path_l.append(slpoint.l)
            for l in sl_frame.aggregated_boundary_low:
                sl_aggregated_boundary_low_l.append(l)
            for l in sl_frame.aggregated_boundary_high:
                sl_aggregated_boundary_high_l.append(l)
            for s in sl_frame.aggregated_boundary_s:
                sl_aggregated_boundary_s.append(s)

        self.sl_data_lock.acquire()
        self.sl_sampled_s = sl_sampled_s
        self.sl_map_upper_boundary = sl_map_upper_boundary
        self.sl_map_lower_boundary = sl_map_lower_boundary
        self.sl_static_obstacle_lower_boundary = sl_static_obstacle_lower_boundary
        self.sl_static_obstacle_upper_boundary = sl_static_obstacle_upper_boundary
        self.sl_dynamic_obstacle_lower_boundary = sl_dynamic_obstacle_lower_boundary
        self.sl_dynamic_obstacle_upper_boundary = sl_dynamic_obstacle_upper_boundary
        self.sl_path_s = sl_path_s
        self.sl_path_l = sl_path_l
        self.sl_aggregated_boundary_low_l = sl_aggregated_boundary_low_l
        self.sl_aggregated_boundary_high_l = sl_aggregated_boundary_high_l
        self.sl_aggregated_boundary_s = sl_aggregated_boundary_s
        self.sl_data_lock.release()

    def compute_st_data(self):
        st_data_boundary_s = {}
        st_data_boundary_t = {}
        st_curve_s = {}
        st_curve_t = {}
        st_curve_v = {}
        st_data_boundary_type = {}
        st_speed_limit_s = {}
        st_speed_limit_v = {}
        st_speed_constraint_s = {}
        st_speed_constraint_lower = {}
        st_speed_constraint_upper = {}
        kernel_cruise_t = {}
        kernel_cruise_s = {}
        kernel_follow_t = {}
        kernel_follow_s = {}

        for st_graph in self.planning_pb.debug.planning_data.st_graph:

            st_data_boundary_s[st_graph.name] = {}
            st_data_boundary_t[st_graph.name] = {}
            st_data_boundary_type[st_graph.name] = {}
            for boundary in st_graph.boundary:
                st_data_boundary_type[st_graph.name][boundary.name] \
                    = planning_internal_pb2.StGraphBoundaryDebug.StBoundaryType.Name(
                    boundary.type)
                st_data_boundary_s[st_graph.name][boundary.name] = []
                st_data_boundary_t[st_graph.name][boundary.name] = []
                for point in boundary.point:
                    st_data_boundary_s[st_graph.name][boundary.name] \
                        .append(point.s)
                    st_data_boundary_t[st_graph.name][boundary.name] \
                        .append(point.t)
                st_data_boundary_s[st_graph.name][boundary.name].append(
                    st_data_boundary_s[st_graph.name][boundary.name][0])
                st_data_boundary_t[st_graph.name][boundary.name].append(
                    st_data_boundary_t[st_graph.name][boundary.name][0])

            st_curve_s[st_graph.name] = []
            st_curve_t[st_graph.name] = []
            st_curve_v[st_graph.name] = []
            for point in st_graph.speed_profile:
                st_curve_s[st_graph.name].append(point.s)
                st_curve_t[st_graph.name].append(point.t)
                st_curve_v[st_graph.name].append(point.v)

            st_speed_limit_s[st_graph.name] = []
            st_speed_limit_v[st_graph.name] = []
            for point in st_graph.speed_limit:
                st_speed_limit_s[st_graph.name].append(point.s)
                st_speed_limit_v[st_graph.name].append(point.v)

            st_speed_constraint_s[st_graph.name] = []
            st_speed_constraint_lower[st_graph.name] = []
            st_speed_constraint_upper[st_graph.name] = []

            speed_constraint = st_graph.speed_constraint
            interp_s_set = []
            for t in speed_constraint.t:
                interp_s = np.interp(t, st_curve_t[st_graph.name],
                                     st_curve_s[st_graph.name])
                interp_s_set.append(interp_s)
            st_speed_constraint_s[st_graph.name].extend(interp_s_set)
            st_speed_constraint_lower[st_graph.name].extend(
                speed_constraint.lower_bound)
            st_speed_constraint_upper[st_graph.name].extend(
                speed_constraint.upper_bound)

            kernel_cruise_t[st_graph.name] = []
            kernel_cruise_s[st_graph.name] = []
            kernel_cruise = st_graph.kernel_cruise_ref
            kernel_cruise_t[st_graph.name].append(kernel_cruise.t)
            kernel_cruise_s[st_graph.name].append(kernel_cruise.cruise_line_s)

            kernel_follow_t[st_graph.name] = []
            kernel_follow_s[st_graph.name] = []
            kernel_follow = st_graph.kernel_follow_ref
            kernel_follow_t[st_graph.name].append(kernel_follow.t)
            kernel_follow_s[st_graph.name].append(kernel_follow.follow_line_s)

        self.st_data_lock.acquire()

        self.st_data_boundary_s = st_data_boundary_s
        self.st_data_boundary_t = st_data_boundary_t
        self.st_curve_s = st_curve_s
        self.st_curve_t = st_curve_t
        self.st_curve_v = st_curve_v
        self.st_speed_limit_v = st_speed_limit_v
        self.st_speed_limit_s = st_speed_limit_s
        self.st_data_boundary_type = st_data_boundary_type

        self.st_speed_constraint_s = st_speed_constraint_s
        self.st_speed_constraint_lower = st_speed_constraint_lower
        self.st_speed_constraint_upper = st_speed_constraint_upper

        self.kernel_cruise_t = kernel_cruise_t
        self.kernel_cruise_s = kernel_cruise_s
        self.kernel_follow_t = kernel_follow_t
        self.kernel_follow_s = kernel_follow_s

        self.st_data_lock.release()

    def compute_traj_data(self):
        traj_speed_t = []
        traj_speed_v = []
        traj_acc_t = []
        traj_acc_a = []
        traj_path_x = []
        traj_path_y = []
        base_time = self.planning_pb.header.timestamp_sec
        for trajectory_point in self.planning_pb.trajectory_point:
            traj_acc_t.append(base_time + trajectory_point.relative_time)
            traj_acc_a.append(trajectory_point.a)
            traj_speed_t.append(base_time + trajectory_point.relative_time)
            traj_speed_v.append(trajectory_point.v)
            traj_path_x.append(trajectory_point.path_point.x)
            traj_path_y.append(trajectory_point.path_point.y)

        self.traj_data_lock.acquire()

        self.traj_speed_t_history.append(traj_speed_t)
        self.traj_speed_v_history.append(traj_speed_v)
        if len(self.traj_speed_t_history) > self.traj_speed_history_len:
            self.traj_speed_t_history = \
                self.traj_speed_t_history[len(self.traj_speed_t_history)
                                          - self.traj_speed_history_len:]
            self.traj_speed_v_history = \
                self.traj_speed_v_history[len(self.traj_speed_v_history)
                                          - self.traj_speed_history_len:]

        self.traj_acc_t_history.append(traj_acc_t)
        self.traj_acc_a_history.append(traj_acc_a)
        if len(self.traj_acc_t_history) > self.traj_acc_history_len:
            self.traj_acc_t_history = \
                self.traj_acc_t_history[len(self.traj_acc_t_history)
                                        - self.traj_acc_history_len:]
            self.traj_acc_a_history = \
                self.traj_acc_a_history[len(self.traj_acc_a_history)
                                        - self.traj_acc_history_len:]

        self.traj_path_x_history.append(traj_path_x)
        self.traj_path_y_history.append(traj_path_y)
        if len(self.traj_path_x_history) > self.traj_path_history_len:
            self.traj_path_x_history = \
                self.traj_path_x_history[len(self.traj_path_x_history)
                                         - self.traj_path_history_len:]
            self.traj_path_y_history = \
                self.traj_path_y_history[len(self.traj_path_y_history)
                                         - self.traj_path_history_len:]

        self.traj_data_lock.release()

    def replot_sl_data(self,
                       sl_static_obstacle_lower_boundary,
                       sl_static_obstacle_upper_boundary,
                       sl_dynamic_obstacle_lower_boundary,
                       sl_dynamic_obstacle_upper_boundary,
                       sl_map_lower_boundary,
                       sl_map_upper_boundary, sl_path,
                       sl_aggregated_boundary_low_line,
                       sl_aggregated_boundary_high_line):
        self.sl_data_lock.acquire()
        sl_static_obstacle_lower_boundary.set_visible(True)
        sl_static_obstacle_upper_boundary.set_visible(True)
        sl_dynamic_obstacle_lower_boundary.set_visible(True)
        sl_dynamic_obstacle_upper_boundary.set_visible(True)
        sl_map_lower_boundary.set_visible(True)
        sl_map_upper_boundary.set_visible(True)
        sl_path.set_visible(True)
        sl_aggregated_boundary_low_line.set_visible(True)
        sl_aggregated_boundary_high_line.set_visible(True)

        new_sampled_s = []
        for s in self.sl_sampled_s:
            new_sampled_s.append(s)
            new_sampled_s.append(s)
        new_map_lower = []
        for l in self.sl_map_lower_boundary:
            new_map_lower.append(l)
            new_map_lower.append(-11)
        new_map_upper = []
        for l in self.sl_map_upper_boundary:
            new_map_upper.append(l)
            new_map_upper.append(11)
        sl_map_lower_boundary.set_xdata(new_sampled_s)
        sl_map_lower_boundary.set_ydata(new_map_lower)
        sl_map_upper_boundary.set_xdata(new_sampled_s)
        sl_map_upper_boundary.set_ydata(new_map_upper)

        sl_dynamic_obstacle_lower_boundary.set_xdata(self.sl_sampled_s)
        sl_dynamic_obstacle_lower_boundary.set_ydata(
            self.sl_dynamic_obstacle_lower_boundary)
        sl_dynamic_obstacle_upper_boundary.set_xdata(self.sl_sampled_s)
        sl_dynamic_obstacle_upper_boundary.set_ydata(
            self.sl_dynamic_obstacle_upper_boundary)

        new_static_lower = []
        for l in self.sl_static_obstacle_lower_boundary:
            new_static_lower.append(l)
            new_static_lower.append(-11)
        new_static_upper = []
        for l in self.sl_static_obstacle_upper_boundary:
            new_static_upper.append(l)
            new_static_upper.append(11)
        sl_static_obstacle_lower_boundary.set_xdata(new_sampled_s)
        sl_static_obstacle_lower_boundary.set_ydata(new_static_lower)
        sl_static_obstacle_upper_boundary.set_xdata(new_sampled_s)
        sl_static_obstacle_upper_boundary.set_ydata(new_static_upper)
        sl_path.set_xdata(self.sl_path_s)
        sl_path.set_ydata(self.sl_path_l)
        sl_aggregated_boundary_low_line.set_xdata(
            self.sl_aggregated_boundary_s)
        sl_aggregated_boundary_low_line.set_ydata(
            self.sl_aggregated_boundary_low_l)
        sl_aggregated_boundary_high_line.set_xdata(
            self.sl_aggregated_boundary_s)
        sl_aggregated_boundary_high_line.set_ydata(
            self.sl_aggregated_boundary_high_l)
        self.sl_data_lock.release()

    def replot_st_data(self, boundaries_pool, st_line,
                       obstacle_annotation_pool, st_graph_name):
        if st_graph_name not in self.st_data_boundary_s:
            return
        if st_graph_name not in self.st_curve_s:
            return

        cnt = 0
        self.st_data_lock.acquire()

        st_graph_boudnary_s = self.st_data_boundary_s[st_graph_name]
        st_graph_boudnary_t = self.st_data_boundary_t[st_graph_name]
        st_boundary_type = self.st_data_boundary_type[st_graph_name]
        for boundary_name in st_graph_boudnary_s.keys():
            if cnt >= len(boundaries_pool):
                print("WARNING: number of path lines is more than "
                      + len(boundaries_pool))
                continue
            boundary = boundaries_pool[cnt]
            boundary.set_visible(True)

            boundary.set_xdata(st_graph_boudnary_t[boundary_name])
            boundary.set_ydata(st_graph_boudnary_s[boundary_name])
            center_t = 0
            center_s = 0
            for i in range(len(st_graph_boudnary_t[boundary_name]) - 1):
                center_s += st_graph_boudnary_s[boundary_name][i]
                center_t += st_graph_boudnary_t[boundary_name][i]
            center_s /= float(len(st_graph_boudnary_s[boundary_name]) - 1)
            center_t /= float(len(st_graph_boudnary_t[boundary_name]) - 1)

            annotation = obstacle_annotation_pool[cnt]
            annotation.set_visible(True)
            annotation.set_text(boundary_name + "_"
                                + st_boundary_type[boundary_name]
                                .replace("ST_BOUNDARY_TYPE_", ""))
            annotation.set_x(center_t)
            annotation.set_y(center_s)

            cnt += 1

        st_line.set_visible(True)
        st_line.set_xdata(self.st_curve_t[st_graph_name])
        st_line.set_ydata(self.st_curve_s[st_graph_name])
        st_line.set_label(st_graph_name[0:5])

        self.st_data_lock.release()

    def compute_path_data(self):
        path_data_x = {}
        path_data_y = {}
        for path_debug in self.planning_pb.debug.planning_data.path:
            name = path_debug.name
            path_data_x[name] = []
            path_data_y[name] = []
            for path_point in path_debug.path_point:
                path_data_x[name].append(path_point.x)
                path_data_y[name].append(path_point.y)
        self.path_data_lock.acquire()
        self.path_data_x = path_data_x
        self.path_data_y = path_data_y
        self.path_data_lock.release()

    def replot_path_data(self, path_lines):
        cnt = 0
        self.path_data_lock.acquire()
        for name in self.path_data_x.keys():
            if cnt >= len(path_lines):
                print("WARNING: number of path lines is more than "
                      + len(path_lines))
                continue
            if len(self.path_data_x[name]) <= 1:
                continue
            line = path_lines[cnt]
            line.set_visible(True)
            line.set_xdata(self.path_data_x[name])
            line.set_ydata(self.path_data_y[name])
            line.set_label(name[0:5])
            cnt += 1
        self.path_data_lock.release()

    def compute_speed_data(self):
        speed_data_time = {}
        speed_data_val = {}

        for speed_plan in self.planning_pb.debug.planning_data.speed_plan:
            name = speed_plan.name
            speed_data_time[name] = []
            speed_data_val[name] = []
            for speed_point in speed_plan.speed_point:
                speed_data_time[name].append(speed_point.t)
                speed_data_val[name].append(speed_point.v)
        name = "final_speed_output"
        speed_data_time[name] = []
        speed_data_val[name] = []
        for traj_point in self.planning_pb.trajectory_point:
            speed_data_time[name].append(traj_point.relative_time)
            speed_data_val[name].append(traj_point.v)
        self.speed_data_lock.acquire()
        self.speed_data_time = speed_data_time
        self.speed_data_val = speed_data_val
        self.speed_data_lock.release()

    def replot_speed_data(self, speed_lines):
        cnt = 0
        self.speed_data_lock.acquire()
        for name in self.speed_data_time.keys():
            if cnt >= len(speed_lines):
                print("WARNING: number of speed lines is more than "
                      + len(speed_lines))
                continue
            if len(self.speed_data_time[name]) <= 1:
                continue
            line = speed_lines[cnt]
            line.set_visible(True)
            line.set_xdata(self.speed_data_time[name])
            line.set_ydata(self.speed_data_val[name])
            line.set_label(name[0:5])
            cnt += 1
        self.speed_data_lock.release()
