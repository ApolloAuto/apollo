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
"""
X Y Item
"""
import math

import numpy as np
from matplotlib import lines
from matplotlib import patches


class Xyitem(object):
    """XY item to plot"""

    def __init__(self, ax, windowsize, vehiclelength, title, xlabel, ylabel):
        self.ax = ax
        self.windowsize = windowsize
        self.vehiclelength = vehiclelength

        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel, fontsize=10)
        self.ax.set_ylabel(ylabel, fontsize=10)

        self.lines = []

        self.pathstartx = []
        self.pathstarty = []

        self.carxhist = []
        self.caryhist = []

        self.targetx = []
        self.targety = []

        self.pathstartidx = -1
        self.carxyhistidx = -1
        self.carposidx = -1
        self.targethistidx = -1

        self.axx = float('inf')
        self.axy = float('inf')

        self.planningavailable = False

    def reset(self):
        """Reset"""
        del self.pathstartx[:]
        del self.pathstarty[:]

        del self.carxhist[:]
        del self.caryhist[:]

        del self.targetx[:]
        del self.targety[:]

        self.ax.cla()

        self.pathstartidx = -1
        self.carxyhistidx = -1
        self.carposidx = -1
        self.targethistidx = -1

        self.axx = float('inf')
        self.axy = float('inf')

        self.planningavailable = False

    def new_planning(self, time, x, y):
        """new planning"""
        self.planningtime = time
        self.planningx = x
        self.planningy = y

        self.pathstartx.append(x[0])
        self.pathstarty.append(y[0])

        if self.pathstartidx == -1:
            self.ax.plot(
                self.pathstartx,
                self.pathstarty,
                color='red',
                marker='*',
                ls='None')
            self.pathstartidx = len(self.ax.lines) - 1
            self.current_line = lines.Line2D(x, y, color='red', lw=1.5)
            self.ax.add_line(self.current_line)
        else:
            self.ax.lines[self.pathstartidx].set_data(self.pathstartx,
                                                      self.pathstarty)
            self.current_line.set_data(x, y)

        self.planningavailable = True

    def new_carstatus(self, time, x, y, heading, steer_angle, autodriving):
        """new carstatus"""
        self.carxhist.append(x)
        self.caryhist.append(y)

        angle = math.degrees(heading) - 90
        carcolor = 'red' if autodriving else 'blue'
        if self.carxyhistidx == -1:
            self.ax.plot(self.carxhist, self.caryhist, color="blue")
            self.carxyhistidx = len(self.ax.lines) - 1

            self.ax.plot(
                self.carxhist,
                self.caryhist,
                marker=(3, 0, angle),
                markersize=20,
                mfc=carcolor)
            self.carposidx = len(self.ax.lines) - 1

        else:
            self.ax.lines[self.carxyhistidx].set_data(self.carxhist,
                                                      self.caryhist)
            self.ax.lines[self.carposidx].set_data(x, y)
            self.ax.lines[self.carposidx].set_marker((3, 0, angle))
            self.ax.lines[self.carposidx].set_mfc(carcolor)
            self.ax.patches[0].remove()

        if self.planningavailable:
            xtarget = np.interp(time, self.planningtime, self.planningx)
            self.targetx.append(xtarget)
            ytarget = np.interp(time, self.planningtime, self.planningy)
            self.targety.append(ytarget)

            if self.targethistidx == -1:
                self.ax.plot(self.targetx, self.targety, color="green", lw=1.5)
                self.targethistidx = len(self.ax.lines) - 1
            else:
                self.ax.lines[self.targethistidx].set_data(
                    self.targetx, self.targety)

        self.ax.add_patch(self.gen_steer_curve(x, y, heading, steer_angle))
        # Update Window X, Y Axis Limits
        xcenter = x + math.cos(heading) * 40
        ycenter = y + math.sin(heading) * 40
        if xcenter >= (self.axx + 20) or xcenter <= (self.axx - 20) or \
                ycenter >= (self.axy + 20) or ycenter <= (self.axy - 20):
            scale = self.ax.get_window_extent(
            )._transform._boxout._bbox.get_points()[1]
            original = self.ax.get_position().get_points()
            finalscale = (original[1] - original[0]) * scale
            ratio = finalscale[1] / finalscale[0]
            self.axx = xcenter
            self.axy = ycenter
            self.ax.set_xlim(
                [xcenter - self.windowsize, xcenter + self.windowsize])
            self.ax.set_ylim([
                ycenter - self.windowsize * ratio,
                ycenter + self.windowsize * ratio
            ])

    def gen_steer_curve(self, x, y, heading, steer_angle):
        """Generate Steering Curve to predict car trajectory"""
        if abs(math.tan(math.radians(steer_angle))) > 0.0001:
            R = self.vehiclelength / math.tan(math.radians(steer_angle))
        else:
            R = 100000

        radius = abs(R)
        lengthangle = 7200 / (2 * math.pi * radius)
        if R >= 0:
            centerangle = math.pi / 2 + heading
            startangle = math.degrees(heading - math.pi / 2)
            theta1 = 0
            theta2 = lengthangle
        else:
            centerangle = heading - math.pi / 2
            startangle = math.degrees(math.pi / 2 + heading)
            theta1 = -lengthangle
            theta2 = 0

        centerx = x + math.cos(centerangle) * radius
        centery = y + math.sin(centerangle) * radius

        curve = patches.Arc(
            (centerx, centery),
            2 * radius,
            2 * radius,
            angle=startangle,
            theta1=theta1,
            theta2=theta2,
            linewidth=2,
            zorder=2)
        return curve

    def draw_lines(self):
        """plot lines"""
        for polygon in self.ax.patches:
            self.ax.draw_artist(polygon)

        for line in self.ax.lines:
            self.ax.draw_artist(line)
