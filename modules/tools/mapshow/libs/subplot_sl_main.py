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


class SlMainSubplot:
    def __init__(self, ax):
        self.ax = ax
        self.sl_static_obstacle_lower_boundary_line, = \
            ax.plot([0], [0], "r-", lw=0.3, alpha=0.8)
        self.sl_static_obstacle_upper_boundary_line, = \
            ax.plot([0], [0], "r-", lw=0.3, alpha=0.8)
        self.sl_dynamic_obstacle_lower_boundary_line, = \
            ax.plot([0], [0], "y-", lw=0.3, alpha=0.8)
        self.sl_dynamic_obstacle_upper_boundary_line, = \
            ax.plot([0], [0], "y-", lw=0.3, alpha=0.8)
        self.sl_map_lower_boundary_line, = \
            ax.plot([0], [0], "b-", lw=0.3, ms=2, alpha=0.8)
        self.sl_map_upper_boundary_line, = \
            ax.plot([0], [0], "b-", lw=0.3, ms=4, alpha=0.8)
        self.sl_path_line, = ax.plot([0], [0], "k--")
        self.sl_aggregated_boundary_low_line, = \
            ax.plot([0], [0], "k-", lw=1, ms=2)
        self.sl_aggregated_boundary_high_line, = \
            ax.plot([0], [0], "k-", lw=1, ms=2)

        ax.set_xlim([-10, 220])
        ax.set_ylim([-2.5, 2.5])
        ax.set_xlabel("s - ref_line (m)")
        ax.set_ylabel("l (m)")
        ax.set_title("QP Path - sl Graph")

        self.set_visible(False)

    def set_visible(self, visible):
        self.sl_static_obstacle_lower_boundary_line.set_visible(visible)
        self.sl_static_obstacle_upper_boundary_line.set_visible(visible)
        self.sl_dynamic_obstacle_lower_boundary_line.set_visible(visible)
        self.sl_dynamic_obstacle_upper_boundary_line.set_visible(visible)
        self.sl_map_lower_boundary_line.set_visible(visible)
        self.sl_map_upper_boundary_line.set_visible(visible)
        self.sl_path_line.set_visible(visible)
        self.sl_aggregated_boundary_low_line.set_visible(visible)
        self.sl_aggregated_boundary_high_line.set_visible(visible)

    def show(self, planning):
        planning.sl_data_lock.acquire()
        self.sl_static_obstacle_lower_boundary_line.set_visible(True)
        self.sl_static_obstacle_upper_boundary_line.set_visible(True)
        self.sl_dynamic_obstacle_lower_boundary_line.set_visible(True)
        self.sl_dynamic_obstacle_upper_boundary_line.set_visible(True)
        self.sl_map_lower_boundary_line.set_visible(True)
        self.sl_map_upper_boundary_line.set_visible(True)
        self.sl_path_line.set_visible(True)
        self.sl_aggregated_boundary_low_line.set_visible(True)
        self.sl_aggregated_boundary_high_line.set_visible(True)

        new_sampled_s = []
        for s in planning.sl_sampled_s:
            new_sampled_s.append(s)
            new_sampled_s.append(s)
        new_map_lower = []
        for l in planning.sl_map_lower_boundary:
            new_map_lower.append(l)
            new_map_lower.append(-11)
        new_map_upper = []
        for l in planning.sl_map_upper_boundary:
            new_map_upper.append(l)
            new_map_upper.append(11)
        self.sl_map_lower_boundary_line.set_xdata(new_sampled_s)
        self.sl_map_lower_boundary_line.set_ydata(new_map_lower)
        self.sl_map_upper_boundary_line.set_xdata(new_sampled_s)
        self.sl_map_upper_boundary_line.set_ydata(new_map_upper)

        self.sl_dynamic_obstacle_lower_boundary_line.set_xdata(
            planning.sl_sampled_s)
        self.sl_dynamic_obstacle_lower_boundary_line.set_ydata(
            planning.sl_dynamic_obstacle_lower_boundary)
        self.sl_dynamic_obstacle_upper_boundary_line.set_xdata(
            planning.sl_sampled_s)
        self.sl_dynamic_obstacle_upper_boundary_line.set_ydata(
            planning.sl_dynamic_obstacle_upper_boundary)

        new_static_lower = []
        for l in planning.sl_static_obstacle_lower_boundary:
            new_static_lower.append(l)
            new_static_lower.append(-11)
        new_static_upper = []
        for l in planning.sl_static_obstacle_upper_boundary:
            new_static_upper.append(l)
            new_static_upper.append(11)
        self.sl_static_obstacle_lower_boundary_line.set_xdata(new_sampled_s)
        self.sl_static_obstacle_lower_boundary_line.set_ydata(new_static_lower)
        self.sl_static_obstacle_upper_boundary_line.set_xdata(new_sampled_s)
        self.sl_static_obstacle_upper_boundary_line.set_ydata(new_static_upper)
        self.sl_path_line.set_xdata(planning.sl_path_s)
        self.sl_path_line.set_ydata(planning.sl_path_l)
        self.sl_aggregated_boundary_low_line.set_xdata(
            planning.sl_aggregated_boundary_s)
        self.sl_aggregated_boundary_low_line.set_ydata(
            planning.sl_aggregated_boundary_low_l)
        self.sl_aggregated_boundary_high_line.set_xdata(
            planning.sl_aggregated_boundary_s)
        self.sl_aggregated_boundary_high_line.set_ydata(
            planning.sl_aggregated_boundary_high_l)
        planning.sl_data_lock.release()
