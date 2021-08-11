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


class ADVehicle:
    def __init__(self):
        self._chassis_pb = None
        self._localization_pb = None
        self.front_edge_to_center = 3.89
        self.back_edge_to_center = 1.043
        self.left_edge_to_center = 1.055
        self.right_edge_to_center = 1.055
        self.speed_mps = None
        self.x = None
        self.y = None
        self.heading = None

    def update_chassis(self, chassis_pb):
        self._chassis_pb = chassis_pb
        self.speed_mps = self._chassis_pb.speed_mps

    def update_localization(self, localization_pb):
        self._localization_pb = localization_pb
        self.x = self._localization_pb.pose.position.x
        self.y = self._localization_pb.pose.position.y
        self.heading = self._localization_pb.pose.heading

    def is_ready(self):
        if self._chassis_pb is None or self._localization_pb is None:
            return False
        return True
