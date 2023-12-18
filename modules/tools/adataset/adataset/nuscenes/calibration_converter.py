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
'''Generate apollo calibration files by nuscenes calibration data.'''

import os
import yaml
import pkgutil
import numpy as np

from pathlib import Path

from adataset.nuscenes.nuscenes import NuScenesSchema, NuScenesHelper, NuScenes
from adataset.nuscenes.geometry import (
  Euler,
  Quaternion,
  euler_to_rotation_matrix,
  rotation_matrix_to_euler
)
from adataset.nuscenes.params import (
  apollo2nuscenes_lidar,
  apollo2nuscenes_camera,
  nuscenes2apollo_imu,
  apollo2nuscenes_radar,
)

CALIBRATION_META_ROOT = 'calibration_meta'
# Lidar meta
VELODYNE_PARAMS_PATH = 'velodyne_params'
LIDAR_NOVATEL_EXTRINSICS = 'lidar_novatel_extrinsics.yaml'
# Camera meta
CAMERA_PARAMS_PATH ='camera_params'
CAMERA_EXTRINSICS = 'camera_extrinsics.yaml'
CAMERA_INTRINSICS = 'camera_intrinsics.yaml'
# Radar meta
RADAR_PARAMS_PATH = 'radar_params'
RADAR_EXTRINSICS = 'radar_extrinsics.yaml'

# Frame ID
CAMERA_FRAME_ID = 'novatel'
RADAR_FRAME_ID = 'novatel'
LIDAR_FRAME_ID = 'novatel'


def _construct_transform(rotation, translation):
  transform = np.identity(4)
  transform[:3, :3] = rotation
  transform[:3, 3] = translation
  return transform

def load_yaml(file_path):
  """Read content from yaml

  Args:
      file_path (str): yaml file

  Returns:
      dict: yaml object
  """
  content = None
  # Read from 'package_data'
  data = pkgutil.get_data('adataset', file_path)
  content = yaml.safe_load(data)
  return content


def save_yaml(file_path, content):
  """Save content to yaml

  Args:
      file_path (src): file path
      content (dict): yaml object
  """
  with open(file_path, 'w') as f:
    yaml.safe_dump(content, f, sort_keys=False)


def gen_camera_extrinsics(child_frame_id, frame_id, calibrated_sensor,
        calibration_file_path):
  """Generate camera extrinsic and intrinsic file

  Args:
      camera_name (str): camera name
      calibrated_sensor (_type_): kitti calibrated_sensor json object
      calibration_file_path (str): saved path
  """
  # 1. Generate extrinsics
  camera_meta_extrinsics = os.path.join(
    CALIBRATION_META_ROOT, CAMERA_PARAMS_PATH, CAMERA_EXTRINSICS)
  camera_extrinsics = load_yaml(camera_meta_extrinsics)
  camera_extrinsics['header']['frame_id'] = frame_id
  camera_extrinsics['child_frame_id'] = child_frame_id
  camera_extrinsics['transform']['translation']['x'] = calibrated_sensor['translation'][0]
  camera_extrinsics['transform']['translation']['y'] = calibrated_sensor['translation'][1]
  camera_extrinsics['transform']['translation']['z'] = calibrated_sensor['translation'][2]
  camera_extrinsics['transform']['rotation']['w'] = calibrated_sensor['rotation'][0]
  camera_extrinsics['transform']['rotation']['x'] = calibrated_sensor['rotation'][1]
  camera_extrinsics['transform']['rotation']['y'] = calibrated_sensor['rotation'][2]
  camera_extrinsics['transform']['rotation']['z'] = calibrated_sensor['rotation'][3]

  file_name = CAMERA_EXTRINSICS.replace('camera', child_frame_id)
  file_path = os.path.join(calibration_file_path, CAMERA_PARAMS_PATH)
  Path(file_path).mkdir(parents=True, exist_ok=True)

  camera_extrinsics_file = os.path.join(file_path, file_name)
  save_yaml(camera_extrinsics_file, camera_extrinsics)


def gen_camera_intrinsics(camera_name, calibrated_sensor, calibration_file_path):
  """Generate camera extrinsic and intrinsic file

  Args:
      camera_name (str): camera name
      calibrated_sensor (_type_): kitti calibrated_sensor json object
      calibration_file_path (str): saved path
  """
  # 2. Generate intrinsics
  camera_meta_intrinsics = os.path.join(
    CALIBRATION_META_ROOT, CAMERA_PARAMS_PATH, CAMERA_INTRINSICS)
  camera_intrinsics = load_yaml(camera_meta_intrinsics)

  camera_intrinsics['header']['frame_id'] = CAMERA_FRAME_ID
  if 'height' in calibrated_sensor:
    camera_intrinsics['height'] = calibrated_sensor['height']
  if 'width' in calibrated_sensor:
    camera_intrinsics['width'] = calibrated_sensor['width']

  camera_intrinsics['K'] = list(calibrated_sensor['K'])
  # camera_intrinsics['D'] = list(calibrated_sensor['D'])
  # camera_intrinsics['R'] =
  # camera_intrinsics['P'] =

  file_name = CAMERA_INTRINSICS.replace('camera', camera_name)
  file_path = os.path.join(calibration_file_path, CAMERA_PARAMS_PATH)
  camera_intrinsics_file = os.path.join(file_path, file_name)
  save_yaml(camera_intrinsics_file, camera_intrinsics)


def gen_radar_params(child_frame_id, frame_id, calibrated_sensor,
        calibration_file_path):
  """Generate radar extrinsic file

  Args:
      radar_name (str): radar name
      calibrated_sensor (_type_): nuscenes calibrated_sensor json object
      calibration_file_path (_type_): saved path
  """
  radar_meta_extrinsics = os.path.join(
    CALIBRATION_META_ROOT, RADAR_PARAMS_PATH, RADAR_EXTRINSICS)
  radar_extrinsics = load_yaml(radar_meta_extrinsics)
  radar_extrinsics['header']['frame_id'] = frame_id
  radar_extrinsics['child_frame_id'] = child_frame_id
  radar_extrinsics['transform']['translation']['x'] = calibrated_sensor['translation'][0]
  radar_extrinsics['transform']['translation']['y'] = calibrated_sensor['translation'][1]
  radar_extrinsics['transform']['translation']['z'] = calibrated_sensor['translation'][2]
  radar_extrinsics['transform']['rotation']['w'] = calibrated_sensor['rotation'][0]
  radar_extrinsics['transform']['rotation']['x'] = calibrated_sensor['rotation'][1]
  radar_extrinsics['transform']['rotation']['y'] = calibrated_sensor['rotation'][2]
  radar_extrinsics['transform']['rotation']['z'] = calibrated_sensor['rotation'][3]

  file_name = RADAR_EXTRINSICS.replace('radar', child_frame_id)
  file_path = os.path.join(calibration_file_path, RADAR_PARAMS_PATH)
  Path(file_path).mkdir(parents=True, exist_ok=True)

  radar_extrinsics_file = os.path.join(file_path, file_name)
  save_yaml(radar_extrinsics_file, radar_extrinsics)


def gen_velodyne_params(child_frame_id, frame_id, calibrated_sensor,
        calibration_file_path):
  """Generate lidar extrinsic file

  Args:
      lidar_name (str): lidar name
      calibrated_sensor (_type_): nuscenes calibrated_sensor json object
      calibration_file_path (str): saved path
  """
  lidar_meta_extrinsics = os.path.join(
    CALIBRATION_META_ROOT, VELODYNE_PARAMS_PATH, LIDAR_NOVATEL_EXTRINSICS)
  lidar_extrinsics = load_yaml(lidar_meta_extrinsics)
  lidar_extrinsics['header']['frame_id'] = frame_id
  lidar_extrinsics['child_frame_id'] = child_frame_id
  lidar_extrinsics['transform']['translation']['x'] = calibrated_sensor['translation'][0]
  lidar_extrinsics['transform']['translation']['y'] = calibrated_sensor['translation'][1]
  lidar_extrinsics['transform']['translation']['z'] = calibrated_sensor['translation'][2]
  lidar_extrinsics['transform']['rotation']['w'] = calibrated_sensor['rotation'][0]
  lidar_extrinsics['transform']['rotation']['x'] = calibrated_sensor['rotation'][1]
  lidar_extrinsics['transform']['rotation']['y'] = calibrated_sensor['rotation'][2]
  lidar_extrinsics['transform']['rotation']['z'] = calibrated_sensor['rotation'][3]

  file_name = LIDAR_NOVATEL_EXTRINSICS.replace('lidar', child_frame_id)
  file_path = os.path.join(calibration_file_path, VELODYNE_PARAMS_PATH)
  Path(file_path).mkdir(parents=True, exist_ok=True)

  lidar_novatel_extrinsics = os.path.join(file_path, file_name)
  save_yaml(lidar_novatel_extrinsics, lidar_extrinsics)


def process_calib_velo_to_imu(channel, calibrated_sensor, calibration_file_path):
  qw, qx, qy, qz = calibrated_sensor['rotation']
  q = Quaternion(qw, qx, qy, qz)
  euler = q.to_euler()
  r_matrix = euler_to_rotation_matrix(euler.roll, euler.pitch, euler.yaw)
  # Apollo coordinate system conversion
  # lidar2imu * n_lidar = n_imu
  # lidar2imu * apollo2n_lidar * apollo_lidar = apollo2n_imu * apollo_imu
  lidar2imu = _construct_transform(r_matrix, calibrated_sensor['translation'])
  velo_to_imu = nuscenes2apollo_imu @ lidar2imu @ apollo2nuscenes_lidar

  r_matrix = velo_to_imu[:3, :3]
  roll, pitch, yaw = rotation_matrix_to_euler(r_matrix)
  q = Euler(roll, pitch, yaw).to_quaternion()
  calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]
  calibrated_sensor['translation'] = velo_to_imu[:3, 3].tolist()
  gen_velodyne_params(channel, LIDAR_FRAME_ID, calibrated_sensor, calibration_file_path)


def process_calib_cam_to_imu(channel, calibrated_sensor, calibration_file_path):
  # 1. camera extrinsics
  qw, qx, qy, qz = calibrated_sensor['rotation']
  q = Quaternion(qw, qx, qy, qz)
  euler = q.to_euler()
  r_matrix = euler_to_rotation_matrix(euler.roll, euler.pitch, euler.yaw)
  # Apollo coordinate system conversion
  # cam2imu * n_cam = n_imu
  # cam2imu * apollo2n_cam * apollo_cam = apollo2n_imu * apollo_imu
  cam2imu = _construct_transform(r_matrix, calibrated_sensor['translation'])
  cam_to_imu = nuscenes2apollo_imu @ cam2imu @ apollo2nuscenes_camera

  r_matrix = cam_to_imu[:3,:3]
  roll, pitch, yaw = rotation_matrix_to_euler(r_matrix)
  q = Euler(roll, pitch, yaw).to_quaternion()
  calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]
  calibrated_sensor['translation'] = cam_to_imu[:3, 3].tolist()
  gen_camera_extrinsics(channel, CAMERA_FRAME_ID, calibrated_sensor,
      calibration_file_path)

  # 2. camera intrinsic
  calibrated_sensor['width'] = 1600
  calibrated_sensor['height'] = 900

  calibrated_sensor['K'] = calibrated_sensor['camera_intrinsic']
  gen_camera_intrinsics(channel, calibrated_sensor, calibration_file_path)


def process_calib_radar_to_imu(channel, calibrated_sensor, calibration_file_path):
  qw, qx, qy, qz = calibrated_sensor['rotation']
  q = Quaternion(qw, qx, qy, qz)
  euler = q.to_euler()
  r_matrix = euler_to_rotation_matrix(euler.roll, euler.pitch, euler.yaw)
  # Apollo coordinate system conversion
  # radar2imu * n_radar = n_imu
  # radar2imu * apollo2n_radar * apollo_radar = apollo2n_imu * apollo_imu
  radar2imu = _construct_transform(r_matrix, calibrated_sensor['translation'])
  radar_to_imu = nuscenes2apollo_imu @ radar2imu @ apollo2nuscenes_radar

  r_matrix = radar_to_imu[:3, :3]
  roll, pitch, yaw = rotation_matrix_to_euler(r_matrix)
  q = Euler(roll, pitch, yaw).to_quaternion()
  calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]
  calibrated_sensor['translation'] = radar_to_imu[:3, 3].tolist()
  gen_radar_params(channel, RADAR_FRAME_ID, calibrated_sensor, calibration_file_path)


def gen_sensor_calibration(calibrations, calibration_file_path):
  """Generate camera/radar/lidar extrinsic file and camera intrinsic file

  Args:
      calibrations (dict): nuscenes calibrated_sensor json objects
      calibration_file_path (str): saved path
  """
  for channel, calibrated_sensor in calibrations.items():
    if channel.startswith('LIDAR'):
      process_calib_velo_to_imu(channel, calibrated_sensor, calibration_file_path)
    elif channel.startswith('CAM'):
      process_calib_cam_to_imu(channel, calibrated_sensor, calibration_file_path)
    elif channel.startswith('RADAR'):
      process_calib_radar_to_imu(channel, calibrated_sensor, calibration_file_path)
    else:
      print("Unsupported sensor: {}".format(channel))


def dataset_to_calibration(nuscenes, calibration_root_path):
  """Generate sensor calibration

  Args:
      nuscenes (_type_): nuscenes(one scene)
      calibration_root_path (str): sensor calibrations saved path
  """
  calibration_file_path = os.path.join(
    calibration_root_path, nuscenes.scene_token)

  calibrations = dict()
  for c, f, ego_pose, calibrated_sensor, t in nuscenes:
    calibrations[c] = calibrated_sensor
  gen_sensor_calibration(calibrations, calibration_file_path)


def convert_calibration(dataset_path, calibration_root_path):
  """Generate sensor calibration

  Args:
      dataset_path (str): nuscenes dataset path
      calibration_root_path (str): sensor calibrations saved path
  """
  nuscenes_schema = NuScenesSchema(dataroot=dataset_path)
  n_helper = NuScenesHelper(nuscenes_schema)

  for scene_token in nuscenes_schema.scene.keys():
    print("Start to convert scene: {}, Pls wait!".format(scene_token))
    nuscenes = NuScenes(n_helper, scene_token)
    dataset_to_calibration(nuscenes, calibration_root_path)
  print("Success! Calibrations saved in '{}'".format(calibration_root_path))
