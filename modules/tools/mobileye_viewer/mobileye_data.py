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

import threading

class MobileyeData:
    def __init__(self, mobileye_pb=None):
        self.mobileye_pb = mobileye_pb
        self.lane_data_lock = threading.Lock()
        self.obstacle_data_lock = threading.Lock()
        self.left_lane_x = []
        self.left_lane_y = []
        self.right_lane_x = []
        self.right_lane_y = []
        self.obstacle_x = []
        self.obstacle_y = []

    def update(self, mobileye_pb):
        self.mobileye_pb = mobileye_pb

    def compute_lanes(self):
        if self.mobileye_pb is None:
            return
        self.left_lane_x = []
        self.left_lane_y = []
        self.right_lane_x = []
        self.right_lane_y = []
        c0 = self.mobileye_pb.lka_768.position
        c1 = self.mobileye_pb.lka_769.heading_angle
        c2 = self.mobileye_pb.lka_768.curvature
        c3 = self.mobileye_pb.lka_768.curvature_derivative
        rangex = self.mobileye_pb.lka_769.view_range
        self.lane_data_lock.acquire()
        for y in range(int(rangex)):
            self.right_lane_y.append(y)
            x = c3*(y*y*y) + c2*(y*y) + c1*y + c0
            self.right_lane_x.append(x)
        self.lane_data_lock.release()

        c0 = self.mobileye_pb.lka_766.position
        c1 = self.mobileye_pb.lka_767.heading_angle
        c2 = self.mobileye_pb.lka_766.curvature
        c3 = self.mobileye_pb.lka_766.curvature_derivative
        rangex = self.mobileye_pb.lka_767.view_range
        print "c0, c1, c2. c3: ", c0, ",", c1, ",", c2, ",", c3
        self.lane_data_lock.acquire()
        for y in range(int(rangex)):
            self.left_lane_y.append(y)
            x = c3*(y * y * y) + c2 * (y * y) + c1 * y + c0
            self.left_lane_x.append(x)
        self.lane_data_lock.release()

        #print self.right_lane_y

    def compute_obstacles(self):
        if self.mobileye_pb is None:
            return

        self.obstacle_data_lock.acquire()
        self.obstacle_x = []
        self.obstacle_y = []
        for i in range(len(self.mobileye_pb.details_739)):
            x = self.mobileye_pb.details_739[i].obstacle_pos_x
            y = self.mobileye_pb.details_739[i].obstacle_pos_y
            self.obstacle_x.append(x)
            self.obstacle_y.append(y*-1)
        self.obstacle_data_lock.release()