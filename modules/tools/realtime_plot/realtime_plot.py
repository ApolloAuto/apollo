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
Real Time Plotting of planning and control
"""
import argparse
import json
import matplotlib.pyplot as plt
import math
import numpy
import rospy
import sys
import tf
import threading
from std_msgs.msg import String
import gflags
from gflags import FLAGS

from item import Item
from modules.localization.proto import localization_pb2
from modules.canbus.proto import chassis_pb2
from modules.planning.proto import planning_pb2
from stitem import Stitem
from xyitem import Xyitem

VehicleLength = 2.85
HistLine2display = 2  #The number of lines to display
MaxSteerAngle = 470  #Maximum Steering Angle
SteerRatio = 16
WindowSize = 80

FLAGS = gflags.FLAGS
gflags.DEFINE_boolean('show_heading', False,
                      'Show heading instead of acceleration')
gflags.DEFINE_boolean('show_st_graph', False,
                      'Show st graph instead of curvature.')


class Plotter(object):
    """
    Plotter Class
    """

    def __init__(self, ax1, ax2, ax3, ax4):
        self.ax1 = ax1
        self.ax2 = ax2
        self.ax3 = ax3
        self.ax4 = ax4

        self.updategraph = False
        self.planningavailable = False

        self.closed = False
        self.carspeed = 0.0
        self.steer_angle = 0.0
        self.autodrive = False
        self.carcurvature = 0.0

        self.lock = threading.Lock()

    def callback_planning(self, data):
        """
        New Planning Trajectory
        """
        entity = planning_pb2.ADCTrajectory()
        entity.CopyFrom(data)
        basetime = entity.header.timestamp_sec
        numpoints = len(entity.adc_trajectory_point)
        if numpoints == 0:
            print entity
            return

        pointx = numpy.zeros(numpoints)
        pointy = numpy.zeros(numpoints)
        pointspeed = numpy.zeros(numpoints)
        pointtime = numpy.zeros(numpoints)
        pointtheta = numpy.zeros(numpoints)
        pointcur = numpy.zeros(numpoints)
        pointacc = numpy.zeros(numpoints)
        for idx in range(numpoints):
            pointx[idx] = entity.adc_trajectory_point[idx].x
            pointy[idx] = entity.adc_trajectory_point[idx].y
            pointspeed[idx] = entity.adc_trajectory_point[idx].speed
            pointtheta[idx] = entity.adc_trajectory_point[idx].theta
            pointcur[idx] = entity.adc_trajectory_point[idx].curvature
            pointacc[idx] = entity.adc_trajectory_point[idx].acceleration_s
            pointtime[
                idx] = entity.adc_trajectory_point[idx].relative_time + basetime

        st_available = False
        debug = entity.debug.debug_message
        for item in debug:
            if item.id == 0:
                stgraph = json.loads(item.info)["st_graph_info"]
                boundaries = stgraph["boundaries"]
                unit_t = stgraph["unit_t"]
                graph_total_t = stgraph["graph_total_t"]
                graph_total_s = stgraph["graph_total_s"]
                st_points = stgraph["st_points"]
                st_a = [point["a"] for point in st_points]
                st_v = [point["v"] for point in st_points]
                st_s = [point["point"]["s"] for point in st_points]
                st_t = [point["point"]["t"] for point in st_points]
                st_available = True

        with self.lock:
            self.ax1.new_planning(pointtime, pointx, pointy)
            self.ax2.new_planning(pointtime, pointspeed)

            if self.ax3.title == "Curvature":
                self.ax3.new_planning(pointtime, pointcur)
            elif st_available:
                self.ax3.new_planning(st_t, st_s, graph_total_t, graph_total_s)

            if self.ax4.title == "Heading":
                self.ax4.new_planning(pointtime, pointtheta)
            else:
                self.ax4.new_planning(pointtime, pointacc)

    def callback_chassis(self, data):
        """
        New localization pose
        """
        entity = chassis_pb2.Chassis()
        entity.CopyFrom(data)
        self.carspeed = entity.speed_mps
        self.steer_angle = entity.steering_percentage / 100 * MaxSteerAngle / SteerRatio

        self.autodrive = (
            entity.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE)
        self.carcurvature = math.tan(
            math.radians(self.steer_angle)) / VehicleLength

    def callback_localization(self, data):
        """
        New localization pose
        """
        entity = localization_pb2.LocalizationEstimate()
        entity.CopyFrom(data)
        quat = (entity.pose.orientation.qx, entity.pose.orientation.qy,
                entity.pose.orientation.qz, entity.pose.orientation.qw)
        heading = tf.transformations.euler_from_quaternion(quat)
        carheading = (heading[2] + math.pi / 2 + math.pi) % (
            2 * math.pi) - math.pi
        carx = entity.pose.position.x
        cary = entity.pose.position.y
        cartime = entity.header.timestamp_sec
        with self.lock:
            self.ax1.new_carstatus(cartime, carx, cary, carheading,
                                   self.steer_angle, self.autodrive)
            self.ax2.new_carstatus(cartime, self.carspeed, self.autodrive)
            if self.ax3.title == "Curvature":
                self.ax3.new_carstatus(cartime, self.carcurvature,
                                       self.autodrive)

            if self.ax4.title == "Heading":
                self.ax4.new_carstatus(cartime, carheading, self.autodrive)
            else:
                acc = entity.pose.linear_acceleration_vrf.y
                self.ax4.new_carstatus(cartime, acc, self.autodrive)

    def updatesub(self, planning_sub, localization_sub, chassis_sub):
        """
        update subscriber
        """
        self.planning_sub = planning_sub
        self.localization_sub = localization_sub
        self.chassis_sub = chassis_sub

    def press(self, event):
        """
        Keyboard events during plotting
        """
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
                self.ax1.reset()
                self.ax2.reset()
                self.ax3.reset()
                self.ax4.reset()
            self.updategraph = True

        if event.key == 'b' or event.key == 'B':
            self.planning_sub.unregister()
            self.localization_sub.unregister()
            self.chassis_sub.unregister()


def main(argv):
    """
    Main function
    """
    argv = FLAGS(argv)

    print """
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
    """
    rospy.init_node('realtime_plot', anonymous=True)

    fig = plt.figure()
    ax1 = plt.subplot(2, 2, 1)
    item1 = Xyitem(ax1, WindowSize, VehicleLength, "Trajectory", "X [m]",
                   "Y [m]")

    ax2 = plt.subplot(2, 2, 2)
    item2 = Item(ax2, "Speed", "Time [sec]", "Speed [m/s]", 0, 30)

    if not FLAGS.show_st_graph:
        ax3 = plt.subplot(2, 2, 3, sharex=ax2)
        item3 = Item(ax3, "Curvature", "Time [sec]", "Curvature [m-1]", -0.2,
                     0.2)
    else:
        ax3 = plt.subplot(2, 2, 3)
        item3 = Stitem(ax3, "ST Graph", "Time [sec]", "S [m]")

    ax4 = plt.subplot(2, 2, 4, sharex=ax2)
    if not FLAGS.show_heading:
        item4 = Item(ax4, "Acceleration", "Time [sec]",
                     "Acceleration [m/sec^2]", -5, 5)
    else:
        item4 = Item(ax4, "Heading", "Time [sec]", "Heading [radian]", -4, 4)

    plt.tight_layout(pad=0.20)
    plt.ion()

    plt.show()
    prevtime = 0

    plotter = Plotter(item1, item2, item3, item4)
    fig.canvas.mpl_connect('key_press_event', plotter.press)
    planning_sub = rospy.Subscriber(
        '/apollo/planning',
        planning_pb2.ADCTrajectory,
        plotter.callback_planning,
        queue_size=3)
    localization_sub = rospy.Subscriber(
        '/apollo/localization/pose',
        localization_pb2.LocalizationEstimate,
        plotter.callback_localization,
        queue_size=3)
    chassis_sub = rospy.Subscriber(
        '/apollo/canbus/chassis',
        chassis_pb2.Chassis,
        plotter.callback_chassis,
        queue_size=3)
    plotter.updatesub(planning_sub, localization_sub, chassis_sub)

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
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
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
