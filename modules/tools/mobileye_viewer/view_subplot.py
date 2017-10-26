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

class ViewSubplot:

    def __init__(self, ax):
        #self.ax = ax
        self.right_lane, = ax.plot(
            [-10, 10, -10, 10], [-10, 150, 150, -10],
            'b-', lw=3, alpha=0.4)
        self.left_lane, = ax.plot(
            [0], [0], 'g-', lw=3, alpha=0.4)
        self.obstacles, =  ax.plot(
            [0], [0], 'r.', ms=20, alpha=0.8)
        ax.set_xlim([-10, 10])
        ax.set_ylim([-10, 100])
        ax.relim()
        ax.set_xlabel("lat(m)")
        self.left_lane.set_visible(False)
        self.right_lane.set_visible(False)


    def show(self, mobileye_data):
        self.left_lane.set_visible(True)
        self.right_lane.set_visible(True)

        mobileye_data.lane_data_lock.acquire()
        self.right_lane.set_xdata(mobileye_data.right_lane_x)
        self.right_lane.set_ydata(mobileye_data.right_lane_y)
        self.left_lane.set_xdata(mobileye_data.left_lane_x)
        self.left_lane.set_ydata(mobileye_data.left_lane_y)
        mobileye_data.lane_data_lock.release()
        #self.ax.autoscale_view()
        #self.ax.relim()

        mobileye_data.obstacle_data_lock.acquire()
        self.obstacles.set_ydata(mobileye_data.obstacle_x)
        self.obstacles.set_xdata(mobileye_data.obstacle_y)
        mobileye_data.obstacle_data_lock.release()


