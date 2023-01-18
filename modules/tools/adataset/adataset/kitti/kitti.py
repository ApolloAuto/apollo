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

import os


from adataset.kitti.common import Message, Pose
from adataset.kitti.sensor import Lidar, Camera, IMU
from adataset.kitti.geometry import Euler
from adataset.kitti.proj_helper import latlon2utm


class KITTISchema(object):
  """KITTI schema

  Args:
      object (_type_): _description_
  """
  def __init__(self, dataroot=None) -> None:
    self.dataroot = dataroot
    self.camera_num = 4

  def lidar_schemes(self):
    path_name = 'velodyne_points'
    timestamps = self._read_timestamps(path_name)
    filenames = self._read_filenames(path_name)
    assert len(timestamps) == len(filenames)

    return [Lidar(t, f) for t, f in zip(timestamps, filenames)]

  def camera_schemes(self):
    schemes = dict()
    for camera_id in range(self.camera_num):
      path_name = 'image_{:02d}'.format(camera_id)
      timestamps = self._read_timestamps(path_name)
      filenames = self._read_filenames(path_name)
      assert len(timestamps) == len(filenames)

      schemes[path_name] = [Camera(t, f) for t, f in zip(timestamps, filenames)]
    return schemes

  def imu_schemes(self):
    path_name = 'oxts'
    timestamps = self._read_timestamps(path_name)
    filenames = self._read_filenames(path_name)
    assert len(timestamps) == len(filenames)

    return [IMU(t, f) for t, f in zip(timestamps, filenames)]

  def _read_timestamps(self, file_path, file_name='timestamps.txt'):
    timestamps_file = os.path.join(self.dataroot, file_path, file_name)
    timestamps = []
    with open(timestamps_file, 'r') as f:
      for line in f:
        timestamps.append(line.strip())
    return timestamps

  def _read_filenames(self, file_path, sub_path='data'):
    filenames = []
    absolute_path = os.path.join(self.dataroot, file_path, sub_path)
    for f in os.listdir(absolute_path):
      file_name = os.path.join(absolute_path, f)
      if os.path.isfile(file_name):
        filenames.append(file_name)
    # Need sorted by name
    filenames.sort()
    return filenames


class KITTI(object):
  """KITTI dataset

  Args:
      object (_type_): _description_
  """
  def __init__(self, kitti_schema) -> None:
    self._kitti_schema = kitti_schema
    self._messages = []
    self.read_messages()

  def __iter__(self):
    for message in self._messages:
      yield message

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    pass

  def read_messages(self):
    # read lidar
    for lidar in self._kitti_schema.lidar_schemes():
      msg = Message(channel='velodyne64', timestamp=lidar.timestamp, file_path=lidar.file_path)
      self._messages.append(msg)
    # read imu
    for imu in self._kitti_schema.imu_schemes():
      ego_pose = Pose()
      utm_x, utm_y, _ = latlon2utm(imu.lat, imu.lon)
      ego_pose.set_translation(utm_x, utm_y, 0)
      euler = Euler(imu.roll, imu.pitch, imu.yaw)
      q = euler.to_quaternion()
      ego_pose.set_rotation(q.w, q.x, q.y, q.z)
      msg = Message(channel='imu', timestamp=imu.timestamp, file_path=imu.file_path, ego_pose=ego_pose)
      self._messages.append(msg)
    # read camera
    for camera_name, schemes in self._kitti_schema.camera_schemes().items():
      for camera in schemes:
        msg = Message(channel=camera_name, timestamp=camera.timestamp, file_path=camera.file_path)
        self._messages.append(msg)
    # sort by timestamp
    self._messages.sort(key=lambda msg : msg.timestamp)
