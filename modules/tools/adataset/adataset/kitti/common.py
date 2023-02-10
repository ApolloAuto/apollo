#!/usr/bin/env python3

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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


class Pose(object):
  def __init__(self) -> None:
    self.translation = []
    self.rotation = []

  def set_translation(self, x, y, z):
    self.translation = [x, y, z]

  def set_rotation(self, qw, qx, qy, qz):
    self.rotation = [qw, qx, qy, qz]


class Message(object):
  def __init__(self, channel=None, file_path=None, ego_pose=None,
      calibrated_sensor=None, timestamp=None) -> None:
    self.channel = channel
    self.file_path = file_path
    self.ego_pose = ego_pose
    self.calibrated_sensor = calibrated_sensor
    self.timestamp = timestamp
