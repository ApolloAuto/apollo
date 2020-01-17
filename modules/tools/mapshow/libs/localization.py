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

import math
import threading

import common.proto_utils as proto_utils
from modules.localization.proto.localization_pb2 import LocalizationEstimate


class Localization:
    def __init__(self, localization_pb=None):
        self.localization_pb = localization_pb
        self.localization_data_lock = threading.Lock()

    def update_localization_pb(self, localization_pb):
        self.localization_data_lock.acquire()
        self.localization_pb = localization_pb
        self.localization_data_lock.release()

    def load(self, localization_file_name):
        self.localization_pb = proto_utils.get_pb_from_text_file(
            localization_file_name, LocalizationEstimate())

    def plot_vehicle(self, ax):
        self.plot_vehicle_position(ax)
        self.plot_vehicle_polygon(ax)
        self.show_annotation(ax)

    def replot_vehicle(self, position_line, polygon_line):
        self.localization_data_lock.acquire()
        if self.localization_pb is None:
            self.localization_data_lock.release()
            return
        position_line.set_visible(True)
        polygon_line.set_visible(True)
        self._replot_vehicle_position(position_line)
        self._replot_vehicle_polygon(polygon_line)
        self.localization_data_lock.release()

    def plot_vehicle_position(self, ax):
        loc_x = [self.localization_pb.pose.position.x]
        loc_y = [self.localization_pb.pose.position.y]
        ax.plot(loc_x, loc_y, "bo")

    def _replot_vehicle_position(self, line):
        loc_x = [self.localization_pb.pose.position.x]
        loc_y = [self.localization_pb.pose.position.y]
        line.set_xdata(loc_x)
        line.set_ydata(loc_y)

    def _replot_vehicle_polygon(self, line):
        position = []
        position.append(self.localization_pb.pose.position.x)
        position.append(self.localization_pb.pose.position.y)
        position.append(self.localization_pb.pose.position.z)

        polygon = self.get_vehicle_polygon(position,
                                           self.localization_pb.pose.heading)
        px = []
        py = []
        for point in polygon:
            px.append(point[0])
            py.append(point[1])
        line.set_xdata(px)
        line.set_ydata(py)

    def plot_vehicle_polygon(self, ax):
        position = []
        position.append(self.localization_pb.pose.position.x)
        position.append(self.localization_pb.pose.position.y)
        position.append(self.localization_pb.pose.position.z)

        polygon = self.get_vehicle_polygon(position,
                                           self.localization_pb.pose.heading)
        self.plot_polygon(polygon, ax)

    def show_annotation(self, ax):
        current_t = self.localization_pb.header.timestamp_sec
        content = "t = " + str(current_t) + "\n"
        content += "speed @y = " + str(
            self.localization_pb.pose.linear_velocity.y) + "\n"
        content += "acc @y = " + str(
            self.localization_pb.pose.linear_acceleration_vrf.y)
        lxy = [-80, 80]
        ax.annotate(
            content,
            xy=(self.localization_pb.pose.position.x,
                self.localization_pb.pose.position.y),
            xytext=lxy,
            textcoords='offset points',
            ha='left',
            va='top',
            bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.3),
            arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
            alpha=0.8)

    def plot_polygon(self, polygon, ax):
        px = []
        py = []
        for point in polygon:
            px.append(point[0])
            py.append(point[1])
        ax.plot(px, py, "g-")

    def get_vehicle_polygon(self, position, heading):

        front_edge_to_center = 3.89
        back_edge_to_center = 1.043
        left_edge_to_center = 1.055
        right_edge_to_center = 1.055

        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        #  (p3)  -------- (p0)
        #        | o     |
        #   (p2) -------- (p1)
        p0_x, p0_y = front_edge_to_center, left_edge_to_center
        p1_x, p1_y = front_edge_to_center, -right_edge_to_center
        p2_x, p2_y = -back_edge_to_center, -left_edge_to_center
        p3_x, p3_y = -back_edge_to_center, right_edge_to_center

        p0_x, p0_y = p0_x * cos_h - p0_y * sin_h, p0_x * sin_h + p0_y * cos_h
        p1_x, p1_y = p1_x * cos_h - p1_y * sin_h, p1_x * sin_h + p1_y * cos_h
        p2_x, p2_y = p2_x * cos_h - p2_y * sin_h, p2_x * sin_h + p2_y * cos_h
        p3_x, p3_y = p3_x * cos_h - p3_y * sin_h, p3_x * sin_h + p3_y * cos_h

        [x, y, z] = position
        polygon = []
        polygon.append([p0_x + x, p0_y + y, 0])
        polygon.append([p1_x + x, p1_y + y, 0])
        polygon.append([p2_x + x, p2_y + y, 0])
        polygon.append([p3_x + x, p3_y + y, 0])
        polygon.append([p0_x + x, p0_y + y, 0])
        return polygon
