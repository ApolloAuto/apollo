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
from lanemarker_corrector import LaneMarkerCorrector

class Obstacle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.lane = 0
        self.length = -1
        self.width = -1
        self.rel_speed = -1


class MobileyeProvider:
    def __init__(self):
        self.mobileye_pb = None
        self.right_lane_marker_coef = [1, 0, 0, 0]
        self.left_lane_marker_coef = [1, 0, 0, 0]
        self.right_lane_marker_range = 0
        self.left_lane_marker_range = 0
        self.obstacles = []

    def update(self, mobileye_pb):
        self.mobileye_pb = mobileye_pb
        self.process_lane_markers()

    def process_lane_markers(self):
        rc0 = self.mobileye_pb.lka_768.position
        rc1 = self.mobileye_pb.lka_769.heading_angle
        rc2 = self.mobileye_pb.lka_768.curvature
        rc3 = self.mobileye_pb.lka_768.curvature_derivative
        self.right_lane_marker_range = self.mobileye_pb.lka_769.view_range
        self.right_lane_marker_coef = [rc0, rc1, rc2, rc3]

        lc0 = self.mobileye_pb.lka_766.position
        lc1 = self.mobileye_pb.lka_767.heading_angle
        lc2 = self.mobileye_pb.lka_766.curvature
        lc3 = self.mobileye_pb.lka_766.curvature_derivative
        self.left_lane_marker_range = self.mobileye_pb.lka_767.view_range
        self.left_lane_marker_coef = [lc0, lc1, lc2, lc3]

    def process_obstacles(self):
        if self.mobileye_pb is None:
            return
        self.obstacles = []
        for i in range(len(self.mobileye_pb.details_739)):
            x = self.mobileye_pb.details_739[i].obstacle_pos_x
            y = self.mobileye_pb.details_739[i].obstacle_pos_y
            obstacle = Obstacle(x, y)
            obstacle.rel_speed = self.mobileye_pb.details_739[
                i].obstacle_rel_vel_x
            if i < len(self.mobileye_pb.details_73a):
                obstacle.lane = self.mobileye_pb.details_73a[i].obstacle_lane
                obstacle.length = self.mobileye_pb.details_73a[
                    i].obstacle_length
                obstacle.width = self.mobileye_pb.details_73a[i].obstacle_width
            self.obstacles.append(obstacle)

    def routing_correct(self, routing_provider, localization_provider):
        routing_segment = routing_provider.get_segment()

        vx = localization_provider.localization_pb.pose.position.x
        vy = localization_provider.localization_pb.pose.position.y
        heading = localization_provider.localization_pb.pose.heading
        position = (vx, vy)
        corrector = LaneMarkerCorrector(self.left_lane_marker_coef,
                                        self.right_lane_marker_coef)
        self.left_lane_marker_coef, self.right_lane_marker_coef = \
            corrector.correct(position, heading, routing_segment)
