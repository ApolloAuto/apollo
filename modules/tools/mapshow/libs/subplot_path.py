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

from modules.tools.mapshow.libs.map import Map


class PathSubplot:
    def __init__(self, ax, map_file=None):
        self.ax = ax
        self.map_width = 20
        if map_file is not None:
            map = Map()
            map.load(map_file)
            map.draw_lanes(ax, False, [])
        self.path_lines = []
        self.path_lines_size = 3
        colors = ['b', 'g', 'r', 'k']
        for i in range(self.path_lines_size):
            line, = ax.plot(
                [0], [0],
                colors[i % len(colors)],
                lw=3 + i * 3,
                alpha=0.4)
            self.path_lines.append(line)

        self.vehicle_position_line, = ax.plot([0], [0], 'go', alpha=0.3)
        self.vehicle_polygon_line, = ax.plot([0], [0], 'g-')
        self.init_point_line, = ax.plot([0], [0], 'ro', alpha=0.3)
        self.set_visible(False)
        ax.set_title("PLANNING PATH")

    def set_visible(self, visible):
        for line in self.path_lines:
            line.set_visible(visible)
        self.vehicle_position_line.set_visible(False)
        self.vehicle_polygon_line.set_visible(False)
        self.init_point_line.set_visible(False)

    def show(self, planning, localization):
        cnt = 0
        planning.path_data_lock.acquire()
        for name in planning.path_data_x.keys():
            if cnt >= self.path_lines_size:
                print("WARNING: number of path lines is more than "
                      + str(self.path_lines_size))
                continue
            if len(planning.path_data_x[name]) <= 1:
                continue
            path_line = self.path_lines[cnt]
            path_line.set_visible(True)
            path_line.set_xdata(planning.path_data_x[name])
            path_line.set_ydata(planning.path_data_y[name])
            path_line.set_label(name[0:5])
            cnt += 1
        planning.path_data_lock.release()

        planning.init_point_lock.acquire()
        self.init_point_line.set_xdata(planning.init_point_x)
        self.init_point_line.set_ydata(planning.init_point_y)
        self.init_point_line.set_visible(True)
        planning.init_point_lock.release()

        localization.localization_data_lock.acquire()
        self.draw_vehicle(localization)
        try:
            self.ax.set_xlim(
                localization.localization_pb.pose.position.x - self.map_width,
                localization.localization_pb.pose.position.x + self.map_width)
            self.ax.set_ylim(
                localization.localization_pb.pose.position.y - self.map_width,
                localization.localization_pb.pose.position.y + self.map_width)
        except:
            pass
        localization.localization_data_lock.release()

        self.ax.autoscale_view()
        self.ax.relim()
        self.ax.legend(loc="upper left", borderaxespad=0., ncol=5)
        # self.ax.axis('equal')

    def zoom_in(self):
        if self.map_width > 20:
            self.map_width -= 20

    def zoom_out(self):
        if self.map_width < 200:
            self.map_width += 20

    def draw_vehicle(self, localization):
        if localization.localization_pb is None:
            return
        self.vehicle_position_line.set_visible(True)
        self.vehicle_polygon_line.set_visible(True)

        loc_x = [localization.localization_pb.pose.position.x]
        loc_y = [localization.localization_pb.pose.position.y]
        self.vehicle_position_line.set_xdata(loc_x)
        self.vehicle_position_line.set_ydata(loc_y)

        position = []
        position.append(localization.localization_pb.pose.position.x)
        position.append(localization.localization_pb.pose.position.y)
        position.append(localization.localization_pb.pose.position.z)

        polygon = localization.get_vehicle_polygon(
            position,
            localization.localization_pb.pose.heading)
        px = []
        py = []
        for point in polygon:
            px.append(point[0])
            py.append(point[1])
        self.vehicle_polygon_line.set_xdata(px)
        self.vehicle_polygon_line.set_ydata(py)
