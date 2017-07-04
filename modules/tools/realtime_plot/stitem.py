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
from matplotlib import lines


class Stitem(object):
    """
    Specific item to plot
    """

    def __init__(self, ax, lines2display, title, xlabel, ylabel):
        self.ax = ax
        self.lines2display = lines2display

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

        self.planningavailable = False

    def new_planning(self, time, values, maxtime, maxvalue):
        """
        new planning
        """
        if self.planningavailable == False:
            self.ax.set_xlim([0, maxtime + 1])
            self.ax.set_ylim([0, maxvalue + 10])
            self.ymax = maxvalue
            self.tmax = maxtime

        else:
            self.current_line.set_color('cyan')
            self.current_line.set_linestyle('dashed')
            self.current_line.set_linewidth(1.5)
            self.lines.append(self.current_line)

            xmin, xmax = self.ax.get_xlim()
            if maxtime > xmax:
                self.ax.set_xlim([0, maxtime])

            ymin, ymax = self.ax.get_ylim()
            if maxvalue > ymax:
                self.ax.set_ylim([0, maxvalue + 10])

        self.current_line = lines.Line2D(time, values, color='red', lw=1.5)
        self.ax.add_line(self.current_line)
        self.planningavailable = True

    def draw_lines(self):
        """
        plot lines
        """
        for polygon in self.ax.patches:
            self.ax.draw_artist(polygon)

        for i in range(
                max(0, len(self.lines) - self.lines2display), len(self.lines)):
            self.ax.draw_artist(self.lines[i])

        if self.planningavailable:
            self.ax.draw_artist(self.current_line)
