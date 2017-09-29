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

import rospy
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from modules.planning.proto import planning_pb2
from subplot_st_main import StMainSubplot
from subplot_path import PathSubplot
from subplot_sl_main import SlMainSubplot
from subplot_st_speed import StSpeedSubplot
from subplot_speed import SpeedSubplot
from subplot_lattice_planning_traj import LatticeSubPlot

from localization import Localization
from prediction import Prediction
from planning import Planning

planning = Planning()
localization = Localization()
prediction = Prediction()

def update(frame_number):
    print ""
    print "      ### BEGIN figure update frame"
    lattice_planning_subplot.show(planning, localization, prediction)
    print "      ### END figure update frame"
    print ""

def lattice_planning_callback(planning_pb):
    print ""
    print "      ### BEGIN lattice_planning_callback frame"
    print "      --- update_planning_pb"
    planning.update_planning_pb(planning_pb)
    planning.compute_lattice_path_data()
    print "      --- update_localization_pb"
    localization.update_localization_pb(
        planning_pb.debug.planning_data.adc_position)
    print "      --- update_prediction_pb"
    prediction.update_prediction_pb(
        planning_pb)
    prediction.compute_prediction_path_data()

    print "      --- planning_pb dp_qp trajectory size = " + \
        str(len(planning_pb.debug.planning_data.path))
    print "      --- planning_pb lattice trajectory size = " + \
        str(len(planning_pb.debug.planning_data.trajectory_path))
    #for each_traj in planning_pb.debug.planning_data.trajectory_path:
    #    print "         --- each trajectory point size = " + str(len(each_traj.trajectory_point))

    print "      --- prediction obstacles size = " + \
        str(len(planning_pb.debug.planning_data.prediction_obstacle))

    print "      ### END lattice_planning_callback frame"
    print ""

def add_listener():
    rospy.init_node('st_plot', anonymous=True)
    rospy.Subscriber('/apollo/planning', planning_pb2.ADCTrajectory,
        lattice_planning_callback)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="debug lattice planner")

    add_listener()
    fig = plt.figure()
    #fig.canvas.mpl_connect('key_press_event', press_key)

    ax = plt.subplot2grid((2, 2), (0, 0), rowspan=3, colspan=3)
    lattice_planning_subplot = LatticeSubPlot(ax)

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.axis('equal')
    plt.show()
