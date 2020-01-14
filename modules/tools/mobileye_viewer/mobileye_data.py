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

import threading


class MobileyeData:
    def __init__(self, mobileye_pb=None):
        self.mobileye_pb = mobileye_pb
        self.lane_data_lock = threading.Lock()
        self.next_lane_data_lock = threading.Lock()
        self.obstacle_data_lock = threading.Lock()
        self.left_lane_x = []
        self.left_lane_y = []
        self.right_lane_x = []
        self.right_lane_y = []
        self.obstacle_x = []
        self.obstacle_y = []
        self.ref_lane_x = []
        self.ref_lane_y = []

        self.next_lanes_x = []
        self.next_lanes_y = []

    def update(self, mobileye_pb):
        self.mobileye_pb = mobileye_pb

    def compute_next_lanes(self):
        if self.mobileye_pb is None:
            return

        self.next_lane_data_lock.acquire()
        self.next_lanes_x = []
        self.next_lanes_y = []
        if len(self.mobileye_pb.next_76c) != len(self.mobileye_pb.next_76d):
            print("next lanes output is incomplete!")
            self.next_lane_data_lock.release()
            return
        for i in range(len(self.mobileye_pb.next_76c)):
            lane_x = []
            lane_y = []
            c0 = self.mobileye_pb.next_76c[i].position
            c1 = self.mobileye_pb.next_76d[i].heading_angle
            c2 = self.mobileye_pb.next_76c[i].curvature
            c3 = self.mobileye_pb.next_76c[i].curvature_derivative
            rangex = self.mobileye_pb.next_76d[i].view_range
            for y in range(int(rangex)):
                lane_y.append(y)
                x = c3*(y*y*y) + c2*(y*y) + c1*y + c0
                lane_x.append(x)
            # print rangex
            self.next_lanes_x.append(lane_x)
            self.next_lanes_y.append(lane_y)
        self.next_lane_data_lock.release()

    def compute_lanes(self):
        if self.mobileye_pb is None:
            return
        self.left_lane_x = []
        self.left_lane_y = []
        self.right_lane_x = []
        self.right_lane_y = []
        self.ref_lane_x = []
        self.ref_lane_y = []

        rc0 = self.mobileye_pb.lka_768.position
        rc1 = self.mobileye_pb.lka_769.heading_angle
        rc2 = self.mobileye_pb.lka_768.curvature
        rc3 = self.mobileye_pb.lka_768.curvature_derivative
        rrangex = self.mobileye_pb.lka_769.view_range + 1
        self.lane_data_lock.acquire()
        for y in range(int(rrangex)):
            self.right_lane_y.append(y)
            x = rc3*(y*y*y) + rc2*(y*y) + rc1*y + rc0
            self.right_lane_x.append(x)
        self.lane_data_lock.release()

        lc0 = self.mobileye_pb.lka_766.position
        lc1 = self.mobileye_pb.lka_767.heading_angle
        lc2 = self.mobileye_pb.lka_766.curvature
        lc3 = self.mobileye_pb.lka_766.curvature_derivative
        lrangex = self.mobileye_pb.lka_767.view_range + 1
        self.lane_data_lock.acquire()
        for y in range(int(lrangex)):
            self.left_lane_y.append(y)
            x = lc3*(y * y * y) + lc2 * (y * y) + lc1 * y + lc0
            self.left_lane_x.append(x)
        self.lane_data_lock.release()

        c0 = (rc0 + lc0) // 2
        c1 = (rc1 + lc1) // 2
        c2 = (rc2 + lc2) // 2
        c3 = (rc3 + lc3) // 2
        rangex = (lrangex + rrangex) // 2
        self.lane_data_lock.acquire()
        for y in range(int(rangex)):
            self.ref_lane_y.append(y)
            x = c3 * (y * y * y) + c2 * (y * y) + c1 * y + c0
            self.ref_lane_x.append(x)
        self.lane_data_lock.release()

    def compute_obstacles(self):
        if self.mobileye_pb is None:
            return
        self.obstacle_x = []
        self.obstacle_y = []
        self.obstacle_data_lock.acquire()
        for i in range(len(self.mobileye_pb.details_739)):
            x = self.mobileye_pb.details_739[i].obstacle_pos_x
            y = self.mobileye_pb.details_739[i].obstacle_pos_y
            self.obstacle_x.append(x)
            self.obstacle_y.append(y * -1)
        self.obstacle_data_lock.release()
