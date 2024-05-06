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
'''Generate apollo calibration files by apolloscape calibration data.'''

import os
import yaml
import numpy as np
import pkgutil

from pathlib import Path

from adataset.apolloscape.geometry import Euler, rotation_matrix_to_euler
from adataset.apolloscape.params import scape2apollo_imu, apollo2scape_imu

CALIBRATION_META_ROOT = 'calibration_meta'
# Lidar meta
VELODYNE_PARAMS_PATH = 'velodyne_params'
NOVATEL_PARAMS_PATH = 'novatel_params'
LIDAR_NOVATEL_EXTRINSICS = 'lidar_novatel_extrinsics.yaml'
NOVATEL_EXTRINSICS = 'novatel_localization_extrinsics.yaml'
# Frame ID
LIDAR_FRAME_ID = 'novatel'
NOVETAL_FRAME_ID = 'localization'


def _construct_transform(rotation, translation):
  """Construct transorm matrix

  Args:
      rotation: rotation matrix
      translation: translation vector

  Returns:
      transform: transform matrix
  """
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


def gen_velodyne_params(child_frame_id, frame_id, calibrated_sensor,
      calibration_file_path):
  """Generate lidar extrinsic file

  Args:
      lidar_name (str): lidar name
      calibrated_sensor (_type_): calibrated_sensor json object
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


def gen_novatel_params(child_frame_id, frame_id, calibrated_sensor,
  calibration_file_path):
  """Generate novatel to localization params

  Args:
      child_frame_id: child frame name
      frame_id: frame name
      calibrated_sensor: calibrated sensor
      calibration_file_path: calibration file path
  """

  # use lidar meta as temp value
  lidar_meta_extrinsics = os.path.join(
    CALIBRATION_META_ROOT, VELODYNE_PARAMS_PATH, LIDAR_NOVATEL_EXTRINSICS)
  novatel_extrinsics = load_yaml(lidar_meta_extrinsics)
  novatel_extrinsics['header']['frame_id'] = frame_id
  novatel_extrinsics['child_frame_id'] = child_frame_id
  novatel_extrinsics['transform']['translation']['x'] = calibrated_sensor['translation'][0]
  novatel_extrinsics['transform']['translation']['y'] = calibrated_sensor['translation'][1]
  novatel_extrinsics['transform']['translation']['z'] = calibrated_sensor['translation'][2]
  novatel_extrinsics['transform']['rotation']['w'] = calibrated_sensor['rotation'][0]
  novatel_extrinsics['transform']['rotation']['x'] = calibrated_sensor['rotation'][1]
  novatel_extrinsics['transform']['rotation']['y'] = calibrated_sensor['rotation'][2]
  novatel_extrinsics['transform']['rotation']['z'] = calibrated_sensor['rotation'][3]

  file_path = os.path.join(calibration_file_path, NOVATEL_PARAMS_PATH)
  Path(file_path).mkdir(parents=True, exist_ok=True)

  novatel_extrinsics_file = os.path.join(file_path, NOVATEL_EXTRINSICS)
  save_yaml(novatel_extrinsics_file, novatel_extrinsics)


def process_calib_velo_to_imu(dataset_path, calibration_file_path):
  """Process calibration of lidar to imu

  Args:
      dataset_path: data path
      calibration_file_path: calibration file path
  """
  calibrated_sensor = dict()
  velo_to_imu = scape2apollo_imu

  rotation = velo_to_imu[:3, :3]
  roll, pitch, yaw = rotation_matrix_to_euler(rotation)
  q = Euler(roll, pitch, yaw).to_quaternion()
  calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]
  calibrated_sensor['translation'] = velo_to_imu[:3, 3].tolist()

  gen_velodyne_params('velodyne64', LIDAR_FRAME_ID, calibrated_sensor,
      calibration_file_path)


def process_calib_imu_to_localization(dataset_path, calibration_file_path):
  """Process calibration of imu to localization

  Args:
      dataset_path: dataset path
      calibration_file_path: calibration file path
  """
  calibrated_sensor = dict()
  novatel_to_localization = np.identity(4) 

  rotation = novatel_to_localization[:3, :3]
  roll, pitch, yaw = rotation_matrix_to_euler(rotation)
  q = Euler(roll, pitch, yaw).to_quaternion()
  calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]
  calibrated_sensor['translation'] = novatel_to_localization[:3, 3].tolist()

  gen_novatel_params('novatel', NOVETAL_FRAME_ID, calibrated_sensor,
      calibration_file_path) 

 
def convert_calibration(dataset_path, calibration_root_path):
  """Convert calibration
  """
  process_calib_velo_to_imu(dataset_path, calibration_root_path)
  process_calib_imu_to_localization(dataset_path, calibration_root_path)
