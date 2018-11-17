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

import random

import matplotlib.pyplot as plt
from matplotlib import cm as cmx
from matplotlib import colors as mcolors

import common.proto_utils as proto_utils
from modules.map.proto import map_pb2

class Map:
    def __init__(self):
        self.map_pb = map_pb2.Map()
        self.colors = []
        self.init_colors()

    def init_colors(self):
        color_num = 6
        self.colors = []
        values = range(color_num)
        jet = plt.get_cmap('brg')
        color_norm = mcolors.Normalize(vmin=0, vmax=values[-1])
        scalar_map = cmx.ScalarMappable(norm=color_norm, cmap=jet)
        for val in values:
            color_val = scalar_map.to_rgba(val)
            self.colors.append(color_val)

    def load(self, map_file_name):
        res = proto_utils.get_pb_from_file(map_file_name, self.map_pb)
        return res != None

    def draw_roads(self, ax):
        cnt = 1
        for road in self.map_pb.road:
            color_val = self.colors[cnt % len(self.colors)]
            self.draw_road(ax, road, color_val)
            cnt += 1

    def draw_road(self, ax, road, color_val):
        for section in road.section:
            for edge in section.boundary.outer_polygon.edge:
                for segment in edge.curve.segment:
                    if segment.HasField('line_segment'):
                        px = []
                        py = []
                        for p in segment.line_segment.point:
                            px.append(float(p.x))
                            py.append(float(p.y))
                        ax.plot(px, py, ls='-', c=color_val, alpha = 0.5)

    def draw_lanes(self, ax, is_show_lane_ids, laneids):
        cnt = 1
        for lane in  self.map_pb.lane:
            color_val = self.colors[cnt % len(self.colors)]
            if len(laneids) == 0:
                self._draw_lane_boundary(lane, ax, color_val)
                self._draw_lane_central(lane, ax, color_val)
            else:
                if lane.id.id in laneids:
                    self._draw_lane_boundary(lane, ax, color_val)
                    self._draw_lane_central(lane, ax, color_val)
            if is_show_lane_ids:
                self._draw_lane_id(lane, ax, color_val)
            elif lane.id.id in laneids:
                self._draw_lane_id(lane, ax, color_val)
            cnt += 1

    def _draw_lane_id(self, lane, ax, color_val):
        """draw lane id"""
        labelxys = []
        labelxys.append((40, -40))
        labelxys.append((-40, -40))
        labelxys.append((40, 40))
        labelxys.append((-40, 40))
        has = ['right', 'left', 'right', 'left']
        vas = ['bottom', 'bottom', 'top', 'top']

        idx = random.randint(0, 3)
        lxy = labelxys[idx]
        x, y = self._find_lane_central_point(lane)
        plt.annotate(
            lane.id.id,
            xy=(x, y), xytext=lxy,
            textcoords='offset points', ha=has[idx], va=vas[idx],
            bbox= dict(boxstyle = 'round,pad=0.5', fc=color_val, alpha=0.5),
            arrowprops = dict(arrowstyle = '-|>', connectionstyle = 'arc3,rad=-0.2', fc=color_val, ec=color_val, alpha=0.5))

    @staticmethod
    def _find_lane_central_point(lane):
        segment_idx = len(lane.left_boundary.curve.segment) / 2
        median_segment = lane.left_boundary.curve.segment[segment_idx]
        left_point_idx = len(median_segment.line_segment.point) / 2
        left_median_point = median_segment.line_segment.point[left_point_idx]

        segment_idx = len(lane.right_boundary.curve.segment) / 2
        median_segment = lane.right_boundary.curve.segment[segment_idx]
        right_point_idx = len(median_segment.line_segment.point) / 2
        right_median_point = median_segment.line_segment.point[right_point_idx]

        x = (left_median_point.x + right_median_point.x) / 2
        y = (left_median_point.y + right_median_point.y) / 2

        return x, y

    @staticmethod
    def _get_median_point(points):
        """get_median_point"""
        if len(points) % 2 == 1:
            point = points[len(points) / 2]
            return point.x, point.y
        else:
            point1 = points[len(points) / 2 - 1]
            point2 = points[len(points) / 2]
            return (point1.x + point2.x) / 2.0, (point1.y + point2.y) / 2.0

    @staticmethod
    def _draw_lane_boundary(lane, ax, color_val):
        """draw boundary"""
        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                px = []
                py = []
                for p in curve.line_segment.point:
                    px.append(float(p.x))
                    py.append(float(p.y))
                ax.plot(px, py, ls='-', c=color_val, alpha = 0.5)
        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                px = []
                py = []
                for p in curve.line_segment.point:
                    px.append(float(p.x))
                    py.append(float(p.y))
                ax.plot(px, py, ls='-', c=color_val, alpha = 0.5)

    @staticmethod
    def _draw_lane_central(lane, ax, color_val):
        """draw boundary"""
        for curve in lane.central_curve.segment:
            if curve.HasField('line_segment'):
                px = []
                py = []
                for p in curve.line_segment.point:
                    px.append(float(p.x))
                    py.append(float(p.y))
                ax.plot(px, py, ls=':', c=color_val, alpha = 0.5)

    def draw_signal_lights(self, ax):
        """draw_signal_lights"""
        for signal in self.map_pb.signal:
            for stop_line in signal.stop_line:
                for curve in stop_line.segment:
                    self._draw_signal(curve.line_segment, signal.id.id, ax)

    @staticmethod
    def _draw_signal(line_segment, label, ax):
        """draw a signal"""
        px = []
        py = []
        for p in line_segment.point:
            px.append(float(p.x))
            py.append(float(p.y))
        ax.plot(px, py, 'o-')
        lxy = [random.randint(20, 80) * random.sample([-1, 1], 1)[0],
               random.randint(20, 80) * random.sample([-1, 1], 1)[0]]
        xy = (sum(px)/len(px), sum(py)/len(py))
        plt.annotate(
            label,
            xy = xy, xytext = lxy,
            textcoords = 'offset points',
            bbox = dict(boxstyle = 'round,pad=0.5', fc = 'yellow', alpha = 0.5),
            arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'))
