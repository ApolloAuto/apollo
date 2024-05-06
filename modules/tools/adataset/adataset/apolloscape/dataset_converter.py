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
'''Generate apollo record file by apolloscape data.'''

import os
import logging
import math

from cyber_record.record import Record
from record_msg.builder import (
  PointCloudBuilder,
  LocalizationBuilder,
  TransformBuilder)

from adataset.apolloscape.apolloscape import ApolloScapeSchema, ApolloScape
from adataset.apolloscape.params import scape2apollo_lidar, apollo2scape_imu
from adataset.apolloscape.geometry import (
  Quaternion,
  get_transform_matrix,
  get_quat_and_vector,
  normalize_angle)


LOCALIZATION_TOPIC = '/apollo/localization/pose'
TF_TOPIC= '/tf'
LIDAR_TOPIC = '/apollo/sensor/velodyne64/compensator/PointCloud2'

PCD_FOLDER = "pcd"
POSE_FOLDER = "pose"


def dataset_to_record(apolloscape, record_file_path):
  """Construct record message and save it as record

  Args:
      apolloscape (_type_): apolloscape
      record_root_path (str): record file saved path
  """
  pc_builder = PointCloudBuilder(dim=5)
  localization_builder = LocalizationBuilder()
  transform_builder = TransformBuilder()

  with Record(record_file_path, mode='w') as record:
    for msg in apolloscape:
      c, f, ego_pose, t = msg.channel, msg.file_path, msg.ego_pose, msg.timestamp
      logging.debug("{}, {}, {}, {}".format(c, f, ego_pose, t))
      pb_msg = pc_builder.build_nuscenes(f, 'velodyne', t, scape2apollo_lidar)
      if pb_msg:
        record.write(LIDAR_TOPIC, pb_msg, int(t*1e9))

      # get novatel2world pose
      quat_scape = ego_pose.rotation
      sensor2world_pose = get_transform_matrix(
          [quat_scape[1], quat_scape[2], quat_scape[3], quat_scape[0]], ego_pose.translation)
      novatel2world_pose = sensor2world_pose @ apollo2scape_imu
      rotation, vector = get_quat_and_vector(novatel2world_pose)

      # write pose to record
      quat = Quaternion(rotation[3], rotation[0], rotation[1], rotation[2])
      heading = normalize_angle(quat.to_euler().yaw + math.pi * 0.5)
      pb_msg = localization_builder.build(
        vector, [quat.w, quat.x, quat.y, quat.z], heading, t)
      if pb_msg:
        record.write(LOCALIZATION_TOPIC, pb_msg, int(t*1e9))

      pb_msg = transform_builder.build('world', 'localization',
        vector, [quat.w, quat.x, quat.y, quat.z], t)
      if pb_msg:
        record.write(TF_TOPIC, pb_msg, int(t*1e9))


def convert_dataset(dataset_path, record_path):
  """Generate apollo record file by apolloscape dataset

  Args:
      dataset_path (str): apolloscape dataset path
      record_path (str): record file saved path
  """
  print("Start to convert apolloscape, Pls wait!")
  records_pcd_folder = os.path.join(dataset_path, PCD_FOLDER)
  records_pose_folder = os.path.join(dataset_path, POSE_FOLDER)
  for record_dir in os.listdir(records_pcd_folder):
    # get pcd and pose folder
    pcd_folder = os.path.join(records_pcd_folder, record_dir)
    pose_folder = os.path.join(records_pose_folder, record_dir)
    # get record
    apolloscape_schema = ApolloScapeSchema(pcd_folder, pose_folder)
    apolloscape = ApolloScape(apolloscape_schema)
    record_name = os.path.join(record_path, record_dir + '.record')
    dataset_to_record(apolloscape, record_name)
    print("Success! Records saved in '{}'".format(record_name))
  print("Apolloscape conversion finished.")
