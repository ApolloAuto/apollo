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
Camera Template Generator
"""
import os

import modules.conf_gen.tpl_gen.camera_tpl as tpl
import modules.conf_gen.common.color_print as p

from modules.conf_gen.common import utils


CAMERA_CONF_DIR = 'modules/drivers/camera/conf'
CAMERA_DAG_DIR = 'modules/drivers/camera/dag'
CAMERA_DAG_FILE = 'camera.dag'

DEFAULT_CAMERA_CONF_PARAMS = {
    'pixel_format': 'yuyv',
    'io_method': 'IO_METHOD_MMAP',
    'frame_rate': 10,
    'monochrome': False,
    'brightness': -1,
    'contrast': -1,
    'saturation': -1,
    'sharpness': -1,
    'gain': -1,
    'auto_focus': False,
    'focus': -1,
    'auto_exposure': True,
    'exposure': 100,
    'auto_white_balance': True,
    'white_balance': 4000,
    'bytes_per_pixel': 2,
    'trigger_internal': 0,
    'trigger_fps': 15,
    'device_wait_ms': 2000,
    'spin_rate': 200,
    'output_type': 'RGB'
}


class CameraTplGenerator:
    """ Camera Template Generator
    """

    def __init__(self, ENV):
        self.env = ENV

    def gen(self):
        """ generate all camera conf
        """
        succ, self.camera_conf_tpl = utils.load_yaml_file(self.env, 'camera_conf.tpl.yaml')
        if not succ:
            return False
        if not self.camera_conf_tpl:
            self.camera_conf_tpl = []
            p.warn('camera_conf.tpl.yaml not found')
            return True
        if not self.validate_tpl():
            p.error('Validate camera_conf.tpl.yaml failed.')
            return False
        if not self.gen_conf():
            p.error('Generate camera conf failed.')
            return False
        return True

    def validate_tpl(self):
        """ validate camera conf template
        """
        if not isinstance(self.camera_conf_tpl, list):
            p.error('camera_conf.tpl.yaml format error: must be a list.')
            return False
        for camera in self.camera_conf_tpl:
            if not isinstance(camera, dict):
                p.error('camera_conf.tpl.yaml format error: every camera must be a dict.')
                return False
            for key in ['frame_id', 'camera_dev', 'width', 'height',
                        'raw_channel_name', 'channel_name', 'compress_channel', 'calibration_params']:
                if key not in camera:
                    p.error(f'camera_conf.tpl.yaml format error: {key} not found.')
                    return False
            if not isinstance(camera['calibration_params'], dict):
                p.error(f'camera_conf.tpl.yaml format error: camera calibration_params must be a dict.')
                return False
            for key in ['lidar_channel', 'lidar_frame_id', 'lidar_rotation']:
                if key not in camera['calibration_params']:
                    p.error(f'camera_conf.tpl.yaml format error: calibration_params.{key} not found.')
                    return False
        return True

    def gen_conf(self):
        """ generate camera conf
        """
        conf_dir = os.path.join(self.env['tmp_dir'], CAMERA_CONF_DIR)
        utils.ensure_dir(conf_dir)
        camera_components = ''
        camera_compress_components = ''
        for camera in self.camera_conf_tpl:
            params = dict(DEFAULT_CAMERA_CONF_PARAMS, **camera)
            camera_conf_file = f'{camera["frame_id"]}.pb.txt'
            params['config_file_path'] = os.path.join(self.env['runtime_dir'], CAMERA_CONF_DIR, camera_conf_file)
            utils.dict_format(params)
            content = tpl.CAMERA_CONF_TPL.format(**params)
            with open(os.path.join(conf_dir, camera_conf_file), 'w', encoding='utf-8') as f:
                f.write(content.lstrip())
                p.info('write camera conf: ' + os.path.join(CAMERA_CONF_DIR, camera_conf_file))

            camera_components += tpl.CAMERA_COMPONENT_TPL.format(**params)
            camera_compress_components += tpl.CAMERA_COMPRESS_COMPONENT_TPL.format(**params)

        # write dag file
        dag_dir = os.path.join(self.env['tmp_dir'], CAMERA_DAG_DIR)
        utils.ensure_dir(dag_dir)
        content = tpl.CAMERA_DAG_TPL.format(camera_components=camera_components,
                                            camera_compress_components=camera_compress_components)
        with open(os.path.join(dag_dir, CAMERA_DAG_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write camera dag: ' + os.path.join(CAMERA_DAG_DIR, CAMERA_DAG_FILE))
        return True
