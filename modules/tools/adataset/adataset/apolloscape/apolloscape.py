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

import json
import os

from adataset.apolloscape.common import Message, Pose, read_pose

LIDAR_NAME = "velodyne64"


class ApolloScapeSchema(object):
  """ApolloScape schema

  Args:
      object (_type_): _description_
  """
  def __init__(self, pcd_folder, pose_folder) -> None:
    """Init

    Args:
        pcd_folder: pcd folder
        pose_folder: pose folder
    """
    self.pcd_folder = pcd_folder
    self.pose_folder = pose_folder

  def get_schemas(self):
    """Read lidar and pose schema
    """
    schemas = list()
    files = [p for p in os.listdir(self.pcd_folder) if p.endswith('.bin')]
    for f in files:
      frame = list()
      pcd_file = os.path.join(self.pcd_folder, f)
      pose_file = os.path.join(self.pose_folder, f.split('.')[0] + '_pose.txt')
      schemas.append((pcd_file, pose_file))
    return schemas


class ApolloScape(object):
  """ApolloScape dataset

  Args:
      object (_type_): _description_
  """
  def __init__(self, apolloscape_schema) -> None:
    """Init
    """
    self._apolloscape_schema = apolloscape_schema
    self._messages = []
    self.read_messages()

  def __iter__(self):
    """Iterator

    Yields:
        message: Message
    """
    for message in self._messages:
      yield message

  def __enter__(self):
    """Enter

    Returns:
        self: self instance
    """
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    """Exit, do nothing
    """
    pass

  def read_messages(self):
    """Read messages
    """
    for schema in self._apolloscape_schema.get_schemas():
      timestamp, pose = read_pose(schema[1])
      msg = Message()
      msg.channel = LIDAR_NAME
      msg.file_path = schema[0]
      msg.timestamp = timestamp
      msg.ego_pose = pose
      self._messages.append(msg)
    self._messages.sort(key=lambda msg : msg.timestamp)
