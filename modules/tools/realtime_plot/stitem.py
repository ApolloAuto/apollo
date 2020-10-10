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
"""S T Item"""

import numpy as np
from matplotlib import lines
from matplotlib.patches import Polygon


class Stitem(object):
    """Specific item to plot"""

    def __init__(self, ax, title, xlabel, ylabel):
        self.ax = ax
        self.title = title
        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel, fontsize=10)
        self.ax.set_ylabel(ylabel, fontsize=10)
        self.planningavailable = False

    def reset(self):
        """Reset"""
        self.ax.cla()
        self.ax.set_xlim([-0.1, 0.1])
        self.ax.set_ylim([-0.1, 0.1])

    def new_planning(self, time, values, polygons_t, polygons_s):
        """new planning"""
        max_time = max(time) + 1
        max_value = max(values) + 1
        if self.planningavailable == False:
            self.ax.set_xlim([0, max_time])
            self.ax.set_ylim([0, max_value])
            self.ymax = max_value
            self.tmax = max_time
            self.current_line = lines.Line2D(time, values, color='red', lw=1.5)
            self.ax.add_line(self.current_line)
        else:
            self.current_line.set_data(time, values)
            _, xmax = self.ax.get_xlim()
            if max_time > xmax:
                self.ax.set_xlim([0, max_time])
            _, ymax = self.ax.get_ylim()
            if max_value > ymax:
                self.ax.set_ylim([0, max_value])

        self.ax.patches = []
        for i in range(len(polygons_s)):
            points = np.vstack((polygons_t[i], polygons_s[i])).T
            polygon = Polygon(points)
            self.ax.add_patch(polygon)

        self.planningavailable = True

    def draw_lines(self):
        """plot lines"""
        for polygon in self.ax.patches:
            self.ax.draw_artist(polygon)

        for line in self.ax.lines:
            self.ax.draw_artist(line)
