#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
import shutil

import yaml

from google.protobuf import text_format

from modules.dreamview.proto import preprocess_table_pb2
from modules.tools.common.proto_utils import get_pb_from_text_file

ROOT_DIR = '/apollo/modules/tools/sensor_calibration'

class ConfigYaml(object):
    """generate yaml configuration for next-step calibration service.
    automatically generate calibration configuration yaml file. help user to
    input the initial extrinsics values or path(lidar, camera), intrinsics path(camera),
    sensor name if needs changes.
    """

    def __init__(self, supported_calibrations=['lidar_to_gnss', 'camera_to_lidar']):
        self._task_name = 'unknown'
        self._supported_tasks = supported_calibrations
        self._source_sensor = ''
        self._table_info = preprocess_table_pb2.PreprocessTable()
        print('calibration service now support: {}'.format(self._supported_tasks))

    def load_sample_yaml_file(self, task_name, sample_file=None):
        if sample_file is None:
            sample_file = os.path.join(ROOT_DIR, 'config', task_name + '_sample_config.yaml')
        try:
            with open(sample_file, 'r') as f:
                data = yaml.safe_load(f)
        except IOError:
            raise ValueError(
                'cannot open the sample configure yaml file at {}'.format(sample_file))
        return data

    def _generate_lidar_to_gnss_calibration_yaml(self, in_data, lidar_folder_name, gnss_folder_name):
        in_data['sensor_files_directory'] = os.path.join(
            '.', lidar_folder_name, "")
        in_data['odometry_file'] = os.path.join(
            '.', gnss_folder_name, 'odometry')
        for lidar_config in self._table_info.lidar_config:
            if lidar_config.sensor_name == self._source_sensor:
                in_data['transform']['translation']['x'] = round(
                    lidar_config.translation.x, 4)
                in_data['transform']["translation"]["y"] = round(
                    lidar_config.translation.y, 4)
                in_data['transform']["translation"]["z"] = round(
                    lidar_config.translation.z, 4)
        return in_data

    def _generate_camera_init_param_yaml(self, root_path, in_data):
        init_param_folder = os.path.join(
            ROOT_DIR, 'config/init_params')
        out_param_folder = os.path.join(root_path, 'init_params')
        if os.path.exists(out_param_folder):
            print('folder: %s exists' % out_param_folder)
        else:
            print('create folder: %s' % out_param_folder)
            os.makedirs(out_param_folder)
        camera_config = self._table_info.camera_config
        # copy sample intrinsic yaml file to correct location
        # wait for user input about intrinsics
        sample_intrinsic_yaml = os.path.join(
            init_param_folder, 'sample_intrinsics.yaml')
        intrinsic_data = self.load_sample_yaml_file(self._task_name,
                                                    sample_file=sample_intrinsic_yaml)
        for iter, data in enumerate(camera_config.D):
            intrinsic_data['D'][iter] = data
        for iter, data in enumerate(camera_config.K):
            intrinsic_data['K'][iter] = data
        # dump the intrinsic yaml data
        out_intrinsic_yaml = os.path.join(out_param_folder,
                                          in_data['source_sensor'] + '_intrinsics.yaml')
        try:
            with open(out_intrinsic_yaml, 'w') as f:
                yaml.safe_dump(intrinsic_data, f)
        except IOError:
            raise ValueError('cannot generate the task config yaml '
                             'file at {}'.format(out_intrinsic_yaml))


        # load extrinsics sample yaml, rename source sensor and destination sensor
        sample_extrinsic_yaml = os.path.join(
            init_param_folder, 'sample_extrinsics.yaml')
        extrinsic_data = self.load_sample_yaml_file(self._task_name,
                                                    sample_file=sample_extrinsic_yaml)
        # set up the source_sensor(camera name) to dest sensor(lidar name)
        extrinsic_data['header']['frame_id'] = in_data['destination_sensor']
        extrinsic_data['child_frame_id'] = in_data['source_sensor']
        extrinsic_data['transform']['translation']['x'] = round(
            camera_config.translation.x, 4)
        extrinsic_data['transform']["translation"]["y"] = round(
            camera_config.translation.y, 4)
        extrinsic_data['transform']["translation"]["z"] = round(
            camera_config.translation.z, 4)
        # dump the extrinsic yaml data
        out_extrinsic_yaml = os.path.join(out_param_folder, in_data['source_sensor']
                                          + '_' + in_data['destination_sensor'] + '_extrinsics.yaml')
        try:
            with open(out_extrinsic_yaml, 'w') as f:
                yaml.safe_dump(extrinsic_data, f)
        except IOError:
            raise ValueError('cannot generate the task config yaml '
                             'file at {}'.format(out_extrinsic_yaml))

    def _generate_camera_to_lidar_calibration_yaml(self, in_data):
        in_data['intrinsic'] = os.path.join('.', 'init_params',
                                            in_data['source_sensor'] + '_intrinsics.yaml')
        in_data['extrinsic'] = os.path.join('.', 'init_params', in_data['source_sensor']
                                            + '_' + in_data['destination_sensor'] + '_extrinsics.yaml')

        return in_data

    def get_task_name(self):
        if self._task_name == 'unknown':
            raise ValueError('have not set the task name, the valid task names'
                             'are: {}'.format(self._supported_tasks))
        return self._task_name

    def generate_task_config_yaml(self, task_name, source_sensor, dest_sensor,
                                  source_folder, dest_folder, out_config_file,
                                  in_config_file=None):
        self._task_name = task_name
        self._source_sensor = source_sensor
        out_data = self.load_sample_yaml_file(
            task_name=task_name, sample_file=in_config_file)
        out_data['source_sensor'] = source_sensor
        out_data['destination_sensor'] = dest_sensor
        if task_name not in self._supported_tasks:
            raise ValueError('does not support the calibration task: {}'.format(
                task_name))
        user_config = os.path.join(ROOT_DIR, 'config',
                                   task_name + '_user.config')
        if os.path.exists(user_config):
            try:
                get_pb_from_text_file(user_config, self._table_info)
            except text_format.ParseError:
                print(f'Error: Cannot parse {user_config} as text proto')

        if self._task_name == 'lidar_to_gnss':
            out_data = self._generate_lidar_to_gnss_calibration_yaml(
                out_data, source_folder, dest_folder)

        elif self._task_name == 'camera_to_lidar':
            if source_folder != dest_folder:
                raise ValueError(
                    'camera frame and lidar frame should be in same folder')
            out_root_path = os.path.dirname(out_config_file)
            self._generate_camera_init_param_yaml(out_root_path, out_data)
            out_data = self._generate_camera_to_lidar_calibration_yaml(
                out_data)

        print(out_data)
        try:
            with open(out_config_file, 'w') as f:
                yaml.safe_dump(out_data, f)
        except IOError:
            raise ValueError(
                'cannot generate the task config yaml file at {}'.format(out_config_file))
        return True
