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
'''NuScenes schema class and helper class to read schema data.'''

import json
import os


class NuScenesSchema(object):
  """NuScenes schema

  Args:
      object (_type_): _description_
  """
  def __init__(self,
      dataroot: str = '/data/sets/nuscenes',
      version: str = 'v1.0-mini') -> None:
    self.dataroot = dataroot
    self.version = version
    self.table_names = ['category', 'attribute', 'visibility', 'instance',
      'sensor', 'calibrated_sensor','ego_pose', 'log', 'scene', 'sample',
      'sample_data', 'sample_annotation', 'map']

    self.schema_list = dict()
    self.schema_hash = dict()
    for table_name in self.table_names:
      self.schema_list[table_name] = self._load_data(table_name)
      self.schema_hash[table_name] = self._create_hash(
          self.schema_list[table_name])

  @property
  def category(self):
    return self.schema_hash['category']

  @property
  def attribute(self):
    return self.schema_hash['attribute']

  @property
  def visibility(self):
    return self.schema_hash['visibility']

  @property
  def instance(self):
    return self.schema_hash['instance']

  @property
  def sensor(self):
    return self.schema_hash['sensor']

  @property
  def calibrated_sensor(self):
    return self.schema_hash['calibrated_sensor']

  @property
  def ego_pose(self):
    return self.schema_hash['ego_pose']

  @property
  def log(self):
    return self.schema_hash['log']

  @property
  def scene(self):
    return self.schema_hash['scene']

  @property
  def sample(self):
    return self.schema_hash['sample']

  @property
  def sample_data(self):
    return self.schema_hash['sample_data']

  @property
  def sample_annotation(self):
    return self.schema_hash['sample_annotation']

  @property
  def map(self):
    return self.schema_hash['map']


  def _load_data(self, table_name) -> list:
    file_path = os.path.join(
      self.dataroot, self.version, "{}.json".format(table_name))
    with open(file_path, 'r') as f:
      data = json.load(f)
    return data

  def _create_hash(self, objects) -> dict:
    schema_hash = dict()
    for obj in objects:
      schema_hash[obj['token']] = obj
    return schema_hash


class NuScenesHelper(object):
  """NuScenesHelper help to query data in dataset

  Args:
      object (_type_): _description_
  """
  def __init__(self, nuscenes_schema) -> None:
    self._nuscenes_schema = nuscenes_schema

  def get_sample_by_scene(self, scene_token):
    scene = self.get_scene(scene_token)
    if scene is None:
      return []
    first_sample_token = scene['first_sample_token']
    last_sample_token = scene['last_sample_token']

    samples = []
    cur_sample_token = first_sample_token
    while cur_sample_token != last_sample_token:
      cur_sample = self.get_sample(cur_sample_token)
      samples.append(cur_sample)
      cur_sample_token = cur_sample['next']

    # Add cur_sample_token == last_sample_token
    cur_sample = self.get_sample(cur_sample_token)
    samples.append(cur_sample)
    return samples

  def get_sample_data_by_sample(self, sample_token):
    sample_datas = []
    for sample_data in self._nuscenes_schema.sample_data.values():
      if sample_data['sample_token'] == sample_token:
        sample_datas.append(sample_data)
    return sample_datas

  def get_sweep_data_by_sample(self, sample_data_token):
    sample_data = self._nuscenes_schema.sample_data[sample_data_token]
    if sample_data is None:
      return []

    # add cur
    sweep_datas = [sample_data]
    # add pre
    prev_token = sample_data['prev']
    while prev_token:
      sample_data = self._nuscenes_schema.sample_data[prev_token]
      sweep_datas.insert(0, sample_data)
      prev_token = sample_data['prev']
    # add next
    next_token = sample_data['next']
    while next_token:
      sample_data = self._nuscenes_schema.sample_data[next_token]
      sweep_datas.append(sample_data)
      next_token = sample_data['next']
    return sweep_datas

  def get_scene(self, scene_token):
    return self._nuscenes_schema.scene.get(scene_token)

  def get_sample(self, sample_token):
    return self._nuscenes_schema.sample.get(sample_token)

  def get_ego_pose(self, ego_pose_token):
    return self._nuscenes_schema.ego_pose.get(ego_pose_token)

  def get_calibrated_sensor(self, calibrated_sensor_token):
    return self._nuscenes_schema.calibrated_sensor.get(calibrated_sensor_token)

  def get_sensor(self, sensor_token):
    return self._nuscenes_schema.sensor.get(sensor_token)

  @property
  def dataset_root(self):
    return self._nuscenes_schema.dataroot


class NuScenes(object):
  """NuScenes dataset

  Args:
      object (_type_): _description_
  """
  def __init__(self, nuscenes_helper, scene_token) -> None:
    self.nuscenes_helper = nuscenes_helper
    self.scene_token = scene_token

  def __iter__(self):
    return self.read_messages(self.scene_token)

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    pass

  def read_messages(self, scene_token):
    total_sample_datas = []
    samples = self.nuscenes_helper.get_sample_by_scene(scene_token)
    for sample in samples:
      sample_datas = self.nuscenes_helper.get_sample_data_by_sample(
          sample['token'])
      total_sample_datas.extend(sample_datas)
    total_sample_datas.sort(key=lambda sample_data: sample_data['timestamp'])

    for sample_data in total_sample_datas:
      timestamp = sample_data['timestamp']
      ego_pose_token = sample_data['ego_pose_token']
      ego_pose = self.nuscenes_helper.get_ego_pose(ego_pose_token)

      calibrated_sensor_token = sample_data['calibrated_sensor_token']
      calibrated_sensor = self.nuscenes_helper.get_calibrated_sensor(
          calibrated_sensor_token)

      sensor_token = calibrated_sensor['sensor_token']
      sensor = self.nuscenes_helper.get_sensor(sensor_token)
      channel = sensor['channel']

      file_name = sample_data['filename']
      file_path = os.path.join(self.nuscenes_helper.dataset_root, file_name)
      yield channel, file_path, ego_pose, calibrated_sensor, timestamp
