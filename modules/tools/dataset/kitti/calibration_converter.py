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
'''Generate apollo calibration files by kitti calibration data.'''

import os
import yaml

import numpy as np

from itertools import chain
from pathlib import Path
from geometry import Euler, rotation_matrix_to_euler


CALIBRATION_META_ROOT = '../calibration_meta'
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
CAMERA_FRAME_ID = 'velodyne64'
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
  with open(file_path, 'r') as f:
    content = yaml.safe_load(f)
  return content


def save_yaml(file_path, content):
  """Save content to yaml

  Args:
      file_path (src): file path
      content (dict): yaml object
  """
  with open(file_path, 'w') as f:
    yaml.safe_dump(content, f, sort_keys=False)


def gen_camera_extrinsics(camera_name, calibrated_sensor, calibration_file_path):
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
  camera_intrinsics['D'] = list(calibrated_sensor['D'])
  # camera_intrinsics['R'] =
  # camera_intrinsics['P'] =

  file_name = CAMERA_INTRINSICS.replace('camera', camera_name)
  file_path = os.path.join(calibration_file_path, CAMERA_PARAMS_PATH)
  camera_intrinsics_file = os.path.join(file_path, file_name)
  save_yaml(camera_intrinsics_file, camera_intrinsics)


def gen_radar_params(radar_name, calibrated_sensor, calibration_file_path):
  """Generate radar extrinsic file

  Args:
      radar_name (str): radar name
      calibrated_sensor (_type_): kitti calibrated_sensor json object
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
      calibrated_sensor (_type_): kitti calibrated_sensor json object
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


def process_calib_imu_to_velo(dataset_path, calibration_file_path):
  absolute_path = os.path.join(dataset_path, 'calib_imu_to_velo.txt')
  calibrated_sensor = dict()
  with open(absolute_path, 'r') as f:
    lines = f.readlines()
    r = lines[1].lstrip('R:').split()
    r_array = np.array(r, dtype=np.float32)
    roll, pitch, yaw = rotation_matrix_to_euler(r_array.reshape((3, 3)))
    q = Euler(roll, pitch, yaw).to_quaternion()
    calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]

    t = lines[2].lstrip('T:').split()
    calibrated_sensor['translation'] = np.array(t, dtype=np.float32).tolist()

  gen_velodyne_params('velodyne64', calibrated_sensor, calibration_file_path)


def process_calib_cam_to_cam(dataset_path, calibration_file_path):
  absolute_path = os.path.join(dataset_path, 'calib_cam_to_cam.txt')

  with open(absolute_path, 'r') as f:
    total_lines = f.readlines()

  total_lines = total_lines[2:]
  for i in range(4):
    lines = total_lines[i*8:(i+1)*8]
    calibrated_sensor = dict()
    s = lines[0][len('S_00:'):].strip().split()
    wh = np.array(s, dtype=np.float32).tolist()
    calibrated_sensor['width'] = wh[0]
    calibrated_sensor['height'] = wh[1]

    k = lines[1][len('S_00:'):].strip().split()
    calibrated_sensor['K'] = np.array(k, dtype=np.float32).tolist()

    d = lines[2][len('S_00:'):].strip().split()
    calibrated_sensor['D'] = np.array(d, dtype=np.float32).tolist()

    r = lines[3][len('S_00:'):].strip().split()
    r_array = np.array(r, dtype=np.float32)
    roll, pitch, yaw = rotation_matrix_to_euler(r_array.reshape((3, 3)))
    q = Euler(roll, pitch, yaw).to_quaternion()
    calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]

    t = lines[4][len('S_00:'):].strip().split()
    calibrated_sensor['translation'] = np.array(t, dtype=np.float32).tolist()

    s_rect = lines[5][len('S_rect_01:'):].strip().split()
    s_rect_array = np.array(s_rect, dtype=np.float32).tolist()
    r_rect = lines[6][len('S_rect_01:'):].strip().split()
    r_rect_array = np.array(r_rect, dtype=np.float32).tolist()
    p_rect = lines[7][len('S_rect_01:'):].strip().split()
    p_rect_array = np.array(p_rect, dtype=np.float32).tolist()

    camera_name = 'camera_{:02d}'.format(i)
    gen_camera_intrinsics(camera_name, calibrated_sensor, calibration_file_path)


def process_calib_velo_to_cam(dataset_path, calibration_file_path):
  absolute_path = os.path.join(dataset_path, 'calib_velo_to_cam.txt')
  calibrated_sensor = dict()
  with open(absolute_path, 'r') as f:
    lines = f.readlines()
    r = lines[1].lstrip('R:').split()
    r_array = np.array(r, dtype=np.float32)
    roll, pitch, yaw = rotation_matrix_to_euler(r_array.reshape((3, 3)))
    q = Euler(roll, pitch, yaw).to_quaternion()
    calibrated_sensor['rotation'] = [q.w, q.x, q.y, q.z]

    t = lines[2].lstrip('T:').split()
    calibrated_sensor['translation'] = np.array(t, dtype=np.float32).tolist()

  gen_camera_extrinsics('camera', calibrated_sensor, calibration_file_path)

def convert_calibration(dataset_path, calibration_root_path):
  process_calib_imu_to_velo(dataset_path, calibration_root_path)
  process_calib_velo_to_cam(dataset_path, calibration_root_path)
  process_calib_cam_to_cam(dataset_path, calibration_root_path)
