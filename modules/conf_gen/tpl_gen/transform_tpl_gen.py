#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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
"""
Transform Template Generator
"""
import os
import yaml

import modules.conf_gen.tpl_gen.transform_tpl as tpl
import modules.conf_gen.common.color_print as p

from modules.conf_gen.common import utils


STATIC_TRANSFORM_CONF_DIR = 'modules/transform/conf'
STATIC_TRANSFORM_CONF_FILE = 'static_transform_conf.pb.txt'

EXTRINSIC_PARAMS_CONF_DIR = 'modules/perception/data/params'
EXTRINSIC_PARAMS_CONF_FILE = '{child_frame_id}_extrinsics.yaml'
INTRINSIC_PARAMS_CONF_DIR = 'modules/perception/data/params'
INTRINSIC_PARAMS_CONF_FILE = '{frame_id}_intrinsics.yaml'

LOCALIZATION_PARAMS_CONF_DIR = 'modules/localization/params'
LOCALIZATION_PARAMS_CONF_FILE = 'imu_localization_extrinsics.yaml'
DEFAULT_NOVTEL_TF_FILE = 'modules/localization/msf/params/novatel_localization_extrinsics.yaml'

DEFAULT_IMU_ANT_OFFSET = {
    'imu_gps_translation_x': 0.0,
    'imu_gps_translation_y': 0.0,
    'imu_gps_translation_z': 0.0
}


class TransformTplGenerator:
    """ Transform Template Generator
    """

    def __init__(self, ENV):
        self.env = ENV
        self.imu_ant_offset = None

    def gen(self):
        """ generate transform conf
        """
        # validate localiztion params
        succ, self.localization_params_tpl = utils.load_yaml_file(self.env, 'localization_params.tpl.yaml')
        if not succ:
            return False
        if not self.localization_params_tpl:
            p.warn('localization_params.tpl.yaml not found')
        else:
            if not self.validate_localization_tpl():
                p.error('Validate localization_params.tpl.yaml failed.')
                return False
        # validate lidar params
        succ, self.lidar_params_tpl = utils.load_yaml_file(self.env, 'lidar_params.tpl.yaml')
        if not succ:
            return False
        if not self.lidar_params_tpl:
            self.lidar_params_tpl = {}
            p.warn('lidar_params.tpl.yaml not found')
        else:
            if not self.validate_lidar_tpl():
                p.error('Validate lidar_params.tpl.yaml failed.')
                return False
        # validate camera params
        succ, self.camera_params_tpl = utils.load_yaml_file(self.env, 'camera_params.tpl.yaml')
        if not succ:
            return False
        if not self.camera_params_tpl:
            self.camera_params_tpl = {}
            p.warn('camera_params.tpl.yaml not found')
        else:
            if not self.validate_camera_tpl():
                p.error('Validate camera_params.tpl.yaml failed.')
                return False
        # generate transform conf
        if not self.gen_conf():
            p.error('Generate transform conf failed.')
            return False
        return True

    def validate_localization_tpl(self):
        """ validate transform conf template
        """
        if not isinstance(self.localization_params_tpl, dict):
            p.error('localization_params.tpl.yaml format error: must be a dict.')
            return False
        if 'imu_localization_tranform' not in self.localization_params_tpl:
            p.error('localization_params.tpl.yaml format error: must have imu_localization_tranform.')
            return False
        imu_localization_tranform = self.localization_params_tpl['imu_localization_tranform']
        if 'translation' not in imu_localization_tranform:
            p.error('localization_params.tpl.yaml format error: must have imu_localization_tranform.translation.')
            return False
        for key in ['x', 'y', 'z']:
            if key not in imu_localization_tranform['translation']:
                p.error(f'localization_params.tpl.yaml format error: must have translation.{key}')
                return False
        if 'rotation' not in imu_localization_tranform:
            p.error('localization_params.tpl.yaml format error: must have imu_localization_tranform.rotation.')
            return False
        for key in ['x', 'y', 'z', 'w']:
            if key not in imu_localization_tranform['rotation']:
                p.error(f'localization_params.tpl.yaml format error: must have rotation.{key}')
                return False

        if 'imu_ant_offset' in self.localization_params_tpl:
            if not isinstance(self.localization_params_tpl['imu_ant_offset'], dict):
                p.error('localization_params.tpl.yaml format error: imu_ant_offset must be a dict.')
                return False
            for key in ['imu_gps_translation_x', 'imu_gps_translation_y', 'imu_gps_translation_z']:
                if key not in self.localization_params_tpl['imu_ant_offset']:
                    p.error(f'localization_params.tpl.yaml format error: must have imu_ant_offset.{key}')
                    return False
        return True

    def validate_transform(self, transform):
        """ validate transform params
        Returns:
            error message
        """
        if not isinstance(transform, dict):
            return 'transform must be a dict.'
        if 'rotation' not in transform:
            return 'transform must have rotation.'
        if not isinstance(transform['rotation'], dict):
            return 'rotation must be a dict.'
        for key in ['x', 'y', 'z', 'w']:
            if key not in transform['rotation']:
                return f'must have rotation.{key}'
        if 'translation' not in transform:
            return 'transform must have translation.'
        if not isinstance(transform['translation'], dict):
            return 'translation must be a dict.'
        for key in ['x', 'y', 'z']:
            if key not in transform['translation']:
                return f'must have translation.{key}'
        return None

    def validate_lidar_tpl(self):
        """ validate lidar transform template
        """
        if not isinstance(self.lidar_params_tpl, dict):
            p.error('lidar_params.tpl.yaml format error: must be a dict.')
            return False
        for _, params in self.lidar_params_tpl.items():
            if not isinstance(params, dict):
                p.error('lidar_params.tpl.yaml format error: params must be a dict.')
                return False
            if 'transform' not in params:
                p.error('lidar_params.tpl.yaml format error: params must have transform.')
                return False
            err_msg = self.validate_transform(params['transform'])
            if err_msg:
                p.error('lidar_params.tpl.yaml format error: ' + err_msg)
                return False
        return True

    def validate_camera_tpl(self):
        """ validate camera transform template
        """
        if not isinstance(self.camera_params_tpl, dict):
            p.error('camera_params.tpl.yaml format error: must be a dict.')
            return False
        for _, params in self.camera_params_tpl.items():
            if not isinstance(params, dict):
                p.error('camera_params.tpl.yaml format error: params must be a dict.')
                return False
            if 'extrinsics' not in params:
                p.error('camera_params.tpl.yaml format error: params must have extrinsics.')
                return False
            extrinsics = params['extrinsics']
            if not isinstance(extrinsics, dict):
                p.error('camera_params.tpl.yaml format error: extrinsics must be a dict.')
                return False
            for key in ['parent_frame_id', 'transform']:
                if key not in extrinsics:
                    p.error(f'camera_params.tpl.yaml format error: extrinsics must have {key}.')
                    return False
            err_msg = self.validate_transform(extrinsics['transform'])
            if err_msg:
                p.error('camera_params.tpl.yaml format error: ' + err_msg)
                return False
            if 'intrinsics' not in params:
                p.error('camera_params.tpl.yaml format error: params must have intrinsics.')
                return False
            if not isinstance(params['intrinsics'], dict):
                p.error('camera_params.tpl.yaml format error: intrinsics must be a dict.')
                return False
        return True

    def gen_conf(self):
        """ generate transform conf
        """
        transform_conf_dir = os.path.join(self.env['tmp_dir'], STATIC_TRANSFORM_CONF_DIR)
        utils.ensure_dir(transform_conf_dir)
        transform_content = ''
        # generate localization params
        if self.localization_params_tpl:
            self.imu_localization_tranform = self.localization_params_tpl['imu_localization_tranform']
            self.imu_ant_offset = self.localization_params_tpl.get('imu_ant_offset', DEFAULT_IMU_ANT_OFFSET)
            conf_dir = os.path.join(self.env['tmp_dir'], LOCALIZATION_PARAMS_CONF_DIR)
            utils.ensure_dir(conf_dir)
            localization_params_filepath = os.path.join(LOCALIZATION_PARAMS_CONF_DIR, LOCALIZATION_PARAMS_CONF_FILE)
            with open(os.path.join(conf_dir, LOCALIZATION_PARAMS_CONF_FILE), 'w', encoding='utf-8') as f:
                f.write(tpl.EXTRINSIC_PARAMS_TPL.format(parent_frame_id='localization', child_frame_id='imu',
                        **utils.get_transform_dict(self.imu_localization_tranform)).lstrip())
                p.info('write localization transform conf: ' + localization_params_filepath)
            transform_content += tpl.STATIC_TRANSFORM_CONF_TPL.format(
                parent_frame_id='localization', child_frame_id='imu',
                file_path=os.path.join(self.env['runtime_dir'], localization_params_filepath))
            # add default novatel
            transform_content += tpl.STATIC_TRANSFORM_CONF_TPL.format(
                parent_frame_id='localization', child_frame_id='novatel',
                file_path=os.path.join(self.env['runtime_dir'], DEFAULT_NOVTEL_TF_FILE))
        # generate lidar transform
        if self.lidar_params_tpl:
            conf_dir = os.path.join(self.env['tmp_dir'], EXTRINSIC_PARAMS_CONF_DIR)
            utils.ensure_dir(conf_dir)
            for frame_id, lidar_params in self.lidar_params_tpl.items():
                transform = lidar_params['transform']
                d = dict({
                    'parent_frame_id': 'imu',
                    'child_frame_id': frame_id,
                }, **utils.get_transform_dict(transform))
                content = tpl.EXTRINSIC_PARAMS_TPL.format(**d)
                filename = EXTRINSIC_PARAMS_CONF_FILE.format(**d)
                file_path = os.path.join(EXTRINSIC_PARAMS_CONF_DIR, filename)
                with open(os.path.join(conf_dir, filename), 'w', encoding='utf-8') as f:
                    f.write(content.lstrip())
                    p.info('write lidar transform conf: ' + file_path)
                transform_content += tpl.STATIC_TRANSFORM_CONF_TPL.format(
                    file_path=os.path.join(self.env['runtime_dir'], file_path), **d)
        # generate camera transform
        if self.camera_params_tpl:
            extrinsics_conf_dir = os.path.join(self.env['tmp_dir'], EXTRINSIC_PARAMS_CONF_DIR)
            utils.ensure_dir(conf_dir)
            intrinsics_conf_dir = os.path.join(self.env['tmp_dir'], INTRINSIC_PARAMS_CONF_DIR)
            utils.ensure_dir(intrinsics_conf_dir)
            for frame_id, camera_params in self.camera_params_tpl.items():
                # extrinsics
                extrinsics = camera_params['extrinsics']
                d = dict(
                    {'child_frame_id': frame_id, 'parent_frame_id': extrinsics['parent_frame_id']},
                    **utils.get_transform_dict(extrinsics['transform']))
                content = tpl.EXTRINSIC_PARAMS_TPL.format(**d)
                filename = EXTRINSIC_PARAMS_CONF_FILE.format(**d)
                file_path = os.path.join(EXTRINSIC_PARAMS_CONF_DIR, filename)
                with open(os.path.join(extrinsics_conf_dir, filename), 'w', encoding='utf-8') as f:
                    f.write(content.lstrip())
                    p.info('write extrinsics conf: ' + file_path)
                transform_content += tpl.STATIC_TRANSFORM_CONF_TPL.format(
                    file_path=os.path.join(self.env['runtime_dir'], file_path), **d)
                # intrinsics
                intrinsics = camera_params['intrinsics']
                filename = INTRINSIC_PARAMS_CONF_FILE.format(frame_id=frame_id)
                with open(os.path.join(intrinsics_conf_dir, filename), 'w', encoding='utf-8') as f:
                    yaml.safe_dump(intrinsics, f, sort_keys=False)
                    p.info('write intrinsics conf: ' + os.path.join(INTRINSIC_PARAMS_CONF_DIR, filename))
        # write transform conf
        with open(os.path.join(transform_conf_dir, STATIC_TRANSFORM_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(transform_content.lstrip())
            p.info('write transform conf: ' + os.path.join(STATIC_TRANSFORM_CONF_DIR, STATIC_TRANSFORM_CONF_FILE))
        return True
