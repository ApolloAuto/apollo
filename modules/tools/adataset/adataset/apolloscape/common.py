#!/usr/bin/env python3

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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


def read_pose(pose_file):
  """Read pose and timestamp from file

  Args:
      pose_file: pose file
  """
  pose = Pose()
  timestamp = 0.
  with open(pose_file, 'r') as f:
    data = f.readlines()[0].strip()
    data = [float(e) for e in data.split(' ')]
    pose.set_translation(*data[2:5])
    pose.set_rotation(data[8], data[5], data[6], data[7])
    timestamp = data[1]
  return timestamp, pose


class Pose(object):
  def __init__(self) -> None:
    """Init
    """
    self.translation = []
    self.rotation = []

  def set_translation(self, x, y, z):
    """Set translation
    """
    self.translation = [x, y, z]

  def set_rotation(self, qw, qx, qy, qz):
    """Set rotation
    """
    self.rotation = [qw, qx, qy, qz]


class Message(object):
  def __init__(self, channel=None, file_path=None, ego_pose=None,
      calibrated_sensor=None, timestamp=None) -> None:
    """Init

    Args:
        channel: channel name
        file_path: file path
        ego_pose: ego pose
        calibrated_sensor: sensor name
        timestamp: timestamp
    """
    self.channel = channel
    self.file_path = file_path
    self.ego_pose = ego_pose
    self.calibrated_sensor = calibrated_sensor
    self.timestamp = timestamp
