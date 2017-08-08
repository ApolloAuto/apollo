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
"""
S T Item
"""
import numpy
from matplotlib import lines
from matplotlib.patches import Polygon


class Stitem(object):
    """
    Specific item to plot
    """

    def __init__(self, ax, title, xlabel, ylabel):
        self.ax = ax

        self.title = title
        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel, fontsize=10)
        self.ax.set_ylabel(ylabel, fontsize=10)

        self.lines = []

        self.planningavailable = False

    def reset(self):
        """
        Reset
        """
        del self.lines[:]

        self.ax.cla()

        self.ax.set_xlim([-0.1, 0.1])
        self.ax.set_ylim([-0.1, 0.1])

    def new_planning(self, time, values, polygons_t, polygons_s):
        """
        new planning
        """
        if self.planningavailable == False:
            self.ax.set_xlim([0, max(time) + 1])
            self.ax.set_ylim([0, max(values) + 1])
            self.ymax = max(values) + 1
            self.tmax = max(time) + 1
            self.current_line = lines.Line2D(time, values, color='red', lw=1.5)
            self.ax.add_line(self.current_line)

        else:
            self.current_line.set_data(time, values)

            xmin, xmax = self.ax.get_xlim()
            if max(time) + 1 > xmax:
                self.ax.set_xlim([0, max(time) + 1])

            ymin, ymax = self.ax.get_ylim()
            if max(values) + 1 > ymax:
                self.ax.set_ylim([0, max(values) + 1])

        self.ax.patches = []
        for i in range(len(polygons_s)):
            points = numpy.transpose(
                numpy.vstack((polygons_t[i], polygons_s[i])))
            polygon = Polygon(points)
            self.ax.add_patch(polygon)

        self.planningavailable = True

    def draw_lines(self):
        """
        plot lines
        """
        for polygon in self.ax.patches:
            self.ax.draw_artist(polygon)

        for line in self.ax.lines:
            self.ax.draw_artist(line)
