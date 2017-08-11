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

import threading

class Planning:
    def __init__(self, planning_pb=None):
        self.planning_pb = planning_pb
        self.path_data_lock = threading.Lock()
        self.path_data_x = {}
        self.path_data_y = {}
        self.speed_data_lock = threading.Lock()
        self.speed_data_time = {}
        self.speed_data_val = {}
        self.st_data_lock = threading.Lock()
        self.st_data_s = {}
        self.st_data_t = {}
        self.st_data_boundary_s = {}
        self.st_data_boundary_t = {}

    def update_planning_pb(self, planning_pb):
        self.planning_pb = planning_pb

    def compute_st_data(self):
        st_data_boundary_s = {}
        st_data_boundary_t = {}
        st_data_s = {}
        st_data_t = {}
        for st_graph in self.planning_pb.debug.planning_data.st_graph:
            for boundary in st_graph.boundary:
                st_data_boundary_s[boundary.name] = []
                st_data_boundary_t[boundary.name] = []
                for point in boundary.point:
                    st_data_boundary_s[boundary.name].append(point.s)
                    st_data_boundary_t[boundary.name].append(point.t)
                st_data_boundary_s[boundary.name].append(
                    st_data_boundary_s[boundary.name][0])
                st_data_boundary_t[boundary.name].append(
                    st_data_boundary_t[boundary.name][0])

            st_data_s[st_graph.name] = []
            st_data_t[st_graph.name] = []
            for point in st_graph.speed_profile:
                st_data_s[st_graph.name].append(point.s)
                st_data_t[st_graph.name].append(point.t)
        self.st_data_lock.acquire()
        self.st_data_boundary_s = st_data_boundary_s
        self.st_data_boundary_t = st_data_boundary_t
        self.st_data_s = st_data_s
        self.st_data_t = st_data_t
        self.st_data_lock.release()

    def replot_st_data(self, boundaries, st_lines, obstacle_annotation_pool):
        cnt = 0
        self.st_data_lock.acquire()
        for name in self.st_data_boundary_s.keys():
            if cnt >= len(boundaries):
                print "WARNING: number of path lines is more than " \
                      + len(boundaries)
                continue
            if len(self.st_data_boundary_s[name]) <= 1:
                continue
            boundary = boundaries[cnt]
            boundary.set_visible(True)

            boundary.set_xdata(self.st_data_boundary_t[name])
            boundary.set_ydata(self.st_data_boundary_s[name])
            center_t = 0
            center_s = 0
            for i in range(len(self.st_data_boundary_t[name])-1):
                center_s += self.st_data_boundary_s[name][i]
                center_t += self.st_data_boundary_t[name][i]
            center_s /= float(len(self.st_data_boundary_t[name])-1)
            center_t /= float(len(self.st_data_boundary_t[name])-1)

            annotation = obstacle_annotation_pool[cnt]
            annotation.set_visible(True)
            annotation.set_text(name)
            annotation.set_x(center_t)
            annotation.set_y(center_s)


            cnt += 1

        cnt = 0
        for name in self.st_data_s.keys():
            if cnt >= len(st_lines):
                print "WARNING: number of path lines is more than " \
                      + len(st_lines)
                continue
            if len(self.st_data_s[name]) <= 1:
                continue
            line = st_lines[cnt]
            line.set_visible(True)
            line.set_xdata(self.st_data_t[name])
            line.set_ydata(self.st_data_s[name])
            line.set_label(name)
            cnt += 1
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
                print "WARNING: number of path lines is more than " \
                      + len(path_lines)
                continue
            if len(self.path_data_x[name]) <= 1:
                continue
            line = path_lines[cnt]
            line.set_visible(True)
            line.set_xdata(self.path_data_x[name])
            line.set_ydata(self.path_data_y[name])
            line.set_label(name)
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
                print "WARNING: number of speed lines is more than " \
                      + len(speed_lines)
                continue
            if len(self.speed_data_time[name]) <= 1:
                continue
            line = speed_lines[cnt]
            line.set_visible(True)
            line.set_xdata(self.speed_data_time[name])
            line.set_ydata(self.speed_data_val[name])
            line.set_label(name)
            cnt += 1
        self.speed_data_lock.release()