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

from itertools import chain
from pathlib import Path

from adataset.nuscenes.nuscenes import NuScenesSchema, NuScenesHelper, NuScenes


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


def gen_camera_params(camera_name, calibrated_sensor, calibration_file_path):
  """Generate camera extrinsic and intrinsic file

  Args:
      camera_name (str): camera name
      calibrated_sensor (_type_): nuscenes calibrated_sensor json object
      calibration_file_path (str): saved path
  """
  # 1. Generate extrinsics
  camera_meta_extrinsics = os.path.join(
    CALIBRATION_META_ROOT, CAMERA_PARAMS_PATH, CAMERA_EXTRINSICS)
  camera_extrinsics = load_yaml(camera_meta_extrinsics)
  camera_extrinsics['header']['frame_id'] = CAMERA_FRAME_ID
  camera_extrinsics['child_frame_id'] = camera_name
  camera_extrinsics['transform']['translation']['x'] = calibrated_sensor['translation'][0]
  camera_extrinsics['transform']['translation']['y'] = calibrated_sensor['translation'][1]
  camera_extrinsics['transform']['translation']['z'] = calibrated_sensor['translation'][2]
  camera_extrinsics['transform']['rotation']['w'] = calibrated_sensor['rotation'][0]
  camera_extrinsics['transform']['rotation']['x'] = calibrated_sensor['rotation'][1]
  camera_extrinsics['transform']['rotation']['y'] = calibrated_sensor['rotation'][2]
  camera_extrinsics['transform']['rotation']['z'] = calibrated_sensor['rotation'][3]

  file_name = CAMERA_EXTRINSICS.replace('camera', camera_name)
  file_path = os.path.join(calibration_file_path, CAMERA_PARAMS_PATH)
  Path(file_path).mkdir(parents=True, exist_ok=True)

  camera_extrinsics_file = os.path.join(file_path, file_name)
  save_yaml(camera_extrinsics_file, camera_extrinsics)

  # 2. Generate intrinsics
  camera_meta_intrinsics = os.path.join(
    CALIBRATION_META_ROOT, CAMERA_PARAMS_PATH, CAMERA_INTRINSICS)
  camera_intrinsics = load_yaml(camera_meta_intrinsics)

  raw_camera_intrinsic = calibrated_sensor['camera_intrinsic']
  camera_intrinsics['header']['frame_id'] = CAMERA_FRAME_ID
  camera_intrinsics['K'] = list(chain.from_iterable(raw_camera_intrinsic))

  # Todo(zero): need to complete
  # camera_intrinsics['D'] =
  # camera_intrinsics['R'] =
  # camera_intrinsics['P'] =

  file_name = CAMERA_INTRINSICS.replace('camera', camera_name)
  camera_intrinsics_file = os.path.join(file_path, file_name)
  save_yaml(camera_intrinsics_file, camera_intrinsics)


def gen_radar_params(radar_name, calibrated_sensor, calibration_file_path):
  """Generate radar extrinsic file

  Args:
      radar_name (str): radar name
      calibrated_sensor (_type_): nuscenes calibrated_sensor json object
      calibration_file_path (_type_): saved path
  """
  radar_meta_extrinsics = os.path.join(
    CALIBRATION_META_ROOT, RADAR_PARAMS_PATH, RADAR_EXTRINSICS)
  radar_extrinsics = load_yaml(radar_meta_extrinsics)
  radar_extrinsics['header']['frame_id'] = RADAR_FRAME_ID
  radar_extrinsics['child_frame_id'] = radar_name
  radar_extrinsics['transform']['translation']['x'] = calibrated_sensor['translation'][0]
  radar_extrinsics['transform']['translation']['y'] = calibrated_sensor['translation'][1]
  radar_extrinsics['transform']['translation']['z'] = calibrated_sensor['translation'][2]
  radar_extrinsics['transform']['rotation']['w'] = calibrated_sensor['rotation'][0]
  radar_extrinsics['transform']['rotation']['x'] = calibrated_sensor['rotation'][1]
  radar_extrinsics['transform']['rotation']['y'] = calibrated_sensor['rotation'][2]
  radar_extrinsics['transform']['rotation']['z'] = calibrated_sensor['rotation'][3]

  file_name = RADAR_EXTRINSICS.replace('radar', radar_name)
  file_path = os.path.join(calibration_file_path, RADAR_PARAMS_PATH)
  Path(file_path).mkdir(parents=True, exist_ok=True)

  radar_extrinsics_file = os.path.join(file_path, file_name)
  save_yaml(radar_extrinsics_file, radar_extrinsics)


def gen_velodyne_params(lidar_name, calibrated_sensor, calibration_file_path):
  """Generate lidar extrinsic file

  Args:
      lidar_name (str): lidar name
      calibrated_sensor (_type_): nuscenes calibrated_sensor json object
      calibration_file_path (str): saved path
  """
  lidar_meta_extrinsics = os.path.join(
    CALIBRATION_META_ROOT, VELODYNE_PARAMS_PATH, LIDAR_NOVATEL_EXTRINSICS)
  lidar_extrinsics = load_yaml(lidar_meta_extrinsics)
  lidar_extrinsics['header']['frame_id'] = LIDAR_FRAME_ID
  lidar_extrinsics['child_frame_id'] = lidar_name
  lidar_extrinsics['transform']['translation']['x'] = calibrated_sensor['translation'][0]
  lidar_extrinsics['transform']['translation']['y'] = calibrated_sensor['translation'][1]
  lidar_extrinsics['transform']['translation']['z'] = calibrated_sensor['translation'][2]
  lidar_extrinsics['transform']['rotation']['w'] = calibrated_sensor['rotation'][0]
  lidar_extrinsics['transform']['rotation']['x'] = calibrated_sensor['rotation'][1]
  lidar_extrinsics['transform']['rotation']['y'] = calibrated_sensor['rotation'][2]
  lidar_extrinsics['transform']['rotation']['z'] = calibrated_sensor['rotation'][3]

  file_name = LIDAR_NOVATEL_EXTRINSICS.replace('lidar', lidar_name)
  file_path = os.path.join(calibration_file_path, VELODYNE_PARAMS_PATH)
  Path(file_path).mkdir(parents=True, exist_ok=True)

  lidar_novatel_extrinsics = os.path.join(file_path, file_name)
  save_yaml(lidar_novatel_extrinsics, lidar_extrinsics)


def gen_sensor_calibration(calibrations, calibration_file_path):
  """Generate camera/radar/lidar extrinsic file and camera intrinsic file

  Args:
      calibrations (dict): nuscenes calibrated_sensor json objects
      calibration_file_path (str): saved path
  """
  for channel, calibrated_sensor in calibrations.items():
    if channel.startswith('CAM'):
      gen_camera_params(channel, calibrated_sensor, calibration_file_path)
    elif channel.startswith('LIDAR'):
      gen_velodyne_params(channel, calibrated_sensor, calibration_file_path)
    elif channel.startswith('RADAR'):
      gen_radar_params(channel, calibrated_sensor, calibration_file_path)
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
