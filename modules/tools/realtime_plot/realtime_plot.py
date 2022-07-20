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
"""Real Time Plotting of planning and control"""

import math
import sys
import threading

import gflags
import matplotlib.pyplot as plt
import numpy as np

from cyber.python.cyber_py3 import cyber
from modules.tools.realtime_plot.item import Item
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.tools.realtime_plot.stitem import Stitem
from modules.tools.realtime_plot.xyitem import Xyitem
import modules.tools.common.proto_utils as proto_utils


VehicleLength = 2.85
HistLine2display = 2  # The number of lines to display
MaxSteerAngle = 470  # Maximum Steering Angle
SteerRatio = 16
WindowSize = 80

FLAGS = gflags.FLAGS
gflags.DEFINE_boolean('show_heading', False,
                      'Show heading instead of acceleration')
gflags.DEFINE_boolean('show_st_graph', False, 'Show st graph')


class Plotter(object):
    """Plotter Class"""

    def __init__(self, ax1, ax2, ax3, ax4, stgraph):
        self.ax = [ax1, ax2, ax3, ax4]

        self.updategraph = False
        self.planningavailable = False

        self.closed = False
        self.carspeed = 0.0
        self.steer_angle = 0.0
        self.autodrive = False
        self.carcurvature = 0.0

        self.stgraph = stgraph

        self.lock = threading.Lock()

    def callback_planning(self, data):
        """New Planning Trajectory"""
        if self.stgraph:
            st_s, st_t, polygons_s, polygons_t = proto_utils.flatten(
                data.debug.planning_data.st_graph,
                ['speed_profile.s',
                 'speed_profile.t',
                 'boundary.point.s',
                 'boundary.point.t'])

            with self.lock:
                for i in range(len(st_s)):
                    self.ax[i].new_planning(st_t[i], st_s[i],
                                            polygons_t[i], polygons_s[i])

        else:
            if len(data.trajectory_point) == 0:
                print(data)
                return

            x, y, speed, theta, kappa, acc, relative_time = np.array(
                proto_utils.flatten(data.trajectory_point,
                                    ['path_point.x',
                                     'path_point.y',
                                     'v',
                                     'path_point.theta',
                                     'path_point.kappa',
                                     'a',
                                     'relative_time']))
            relative_time += data.header.timestamp_sec

            with self.lock:
                self.ax[0].new_planning(relative_time, x, y)
                self.ax[1].new_planning(relative_time, speed)

                if self.ax[2].title == "Curvature":
                    self.ax[2].new_planning(relative_time, kappa)

                if self.ax[3].title == "Heading":
                    self.ax[3].new_planning(relative_time, theta)
                else:
                    self.ax[3].new_planning(relative_time, acc)

    def callback_chassis(self, data):
        """New localization pose"""
        if self.stgraph:
            return
        self.carspeed = data.speed_mps
        self.steer_angle = \
            data.steering_percentage / 100 * MaxSteerAngle / SteerRatio

        self.autodrive = (data.driving_mode == Chassis.COMPLETE_AUTO_DRIVE)
        self.carcurvature = math.tan(
            math.radians(self.steer_angle)) / VehicleLength

    def callback_localization(self, data):
        """New localization pose"""
        if self.stgraph:
            return
        carheading = data.pose.heading
        carx = data.pose.position.x
        cary = data.pose.position.y
        cartime = data.header.timestamp_sec
        with self.lock:
            self.ax[0].new_carstatus(cartime, carx, cary, carheading,
                                     self.steer_angle, self.autodrive)
            self.ax[1].new_carstatus(cartime, self.carspeed, self.autodrive)
            self.ax[2].new_carstatus(
                cartime, self.carcurvature, self.autodrive)

            if self.ax[3].title == "Heading":
                self.ax[3].new_carstatus(cartime, carheading, self.autodrive)
            else:
                acc = data.pose.linear_acceleration_vrf.y
                self.ax[3].new_carstatus(cartime, acc, self.autodrive)

    def press(self, event):
        """Keyboard events during plotting"""
        if event.key == 'q' or event.key == 'Q':
            plt.close('all')
            self.closed = True

        if event.key == 'x' or event.key == 'X':
            self.updategraph = True

        if event.key == 'a' or event.key == 'A':
            fig = plt.gcf()
            fig.gca().autoscale()
            fig.canvas.draw()

        if event.key == 'n' or event.key == 'N':
            with self.lock:
                for ax in self.ax:
                    ax.reset()
            self.updategraph = True


def main(argv):
    """Main function"""
    argv = FLAGS(argv)

    print("""
    Keyboard Shortcut:
        [q]: Quit Tool
        [s]: Save Figure
        [a]: Auto-adjust x, y axis to display entire plot
        [x]: Update Figure to Display last few Planning Trajectory instead of all
        [h][r]: Go back Home, Display all Planning Trajectory
        [f]: Toggle Full Screen
        [n]: Reset all Plots
        [b]: Unsubscribe Topics

    Legend Description:
        Red Line: Current Planning Trajectory
        Blue Line: Past Car Status History
        Green Line: Past Planning Target History at every Car Status Frame
        Cyan Dashed Line: Past Planning Trajectory Frames
    """)
    cyber.init()
    planning_sub = cyber.Node("stat_planning")

    fig = plt.figure()

    if not FLAGS.show_st_graph:
        ax1 = plt.subplot(2, 2, 1)
        item1 = Xyitem(ax1, WindowSize, VehicleLength, "Trajectory", "X [m]",
                       "Y [m]")

        ax2 = plt.subplot(2, 2, 2)
        item2 = Item(ax2, "Speed", "Time [sec]", "Speed [m/s]", 0, 30)

        ax3 = plt.subplot(2, 2, 3, sharex=ax2)
        item3 = Item(ax3, "Curvature", "Time [sec]", "Curvature [m-1]", -0.2,
                     0.2)

        ax4 = plt.subplot(2, 2, 4, sharex=ax2)
        if not FLAGS.show_heading:
            item4 = Item(ax4, "Acceleration", "Time [sec]",
                         "Acceleration [m/sec^2]", -5, 5)
        else:
            item4 = Item(ax4, "Heading", "Time [sec]", "Heading [radian]", -4,
                         4)
    else:
        ax1 = plt.subplot(2, 2, 1)
        item1 = Stitem(ax1, "ST Graph", "Time [sec]", "S [m]")

        ax2 = plt.subplot(2, 2, 2)
        item2 = Stitem(ax2, "ST Graph", "Time [sec]", "S [m]")

        ax3 = plt.subplot(2, 2, 3)
        item3 = Stitem(ax3, "ST Graph", "Time [sec]", "S [m]")

        ax4 = plt.subplot(2, 2, 4)
        item4 = Stitem(ax4, "ST Graph", "Time [sec]", "S [m]")

    plt.tight_layout(pad=0.20)
    plt.ion()
    plt.show()

    plotter = Plotter(item1, item2, item3, item4, FLAGS.show_st_graph)
    fig.canvas.mpl_connect('key_press_event', plotter.press)
    planning_sub.create_reader('/apollo/planning',
                               ADCTrajectory, plotter.callback_planning)
    if not FLAGS.show_st_graph:
        localization_sub = cyber.Node("localization_sub")
        localization_sub.create_reader('/apollo/localization/pose',
                                       LocalizationEstimate, plotter.callback_localization)
        chassis_sub = cyber.Node("chassis_sub")
        chassis_sub.create_reader('/apollo/canbus/chassis',
                                  Chassis, plotter.callback_chassis)

    while not cyber.is_shutdown():
        ax1.draw_artist(ax1.patch)
        ax2.draw_artist(ax2.patch)
        ax3.draw_artist(ax3.patch)
        ax4.draw_artist(ax4.patch)

        with plotter.lock:
            item1.draw_lines()
            item2.draw_lines()
            item3.draw_lines()
            item4.draw_lines()

        fig.canvas.blit(ax1.bbox)
        fig.canvas.blit(ax2.bbox)
        fig.canvas.blit(ax3.bbox)
        fig.canvas.blit(ax4.bbox)
        fig.canvas.flush_events()


if __name__ == '__main__':
    main(sys.argv)
