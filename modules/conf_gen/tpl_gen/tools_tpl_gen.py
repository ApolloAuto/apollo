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
Tools Template Generator
"""
import os

import modules.conf_gen.tpl_gen.tools_tpl as tpl
import modules.conf_gen.common.color_print as p
import modules.conf_gen.tpl_gen.transform_tpl_gen as tf_tpl

from modules.conf_gen.common import utils

IMAGE_CREATOR_CONF_DIR = 'modules/private_tools/tile_map_images_creator/conf'
IMAGE_CREATOR_CONF_FILE = 'image_creator_conf.pb.txt'

DEFAULT_IMAGE_CREATOR_PARAMS = {
    'intensity_sigmoid_k': 0.35,
    'intensity_sigmoid_x0': 15.0,
    'upper_height_limit_relative_to_pose': 0.4,
    'lower_height_limit_relative_to_pose': -5.0,
    'upper_distance_limit': 20.0,
    'lower_distance_limit': 5.0,
    'sample_distance': 0.0,
    'enable_height_relative_converter': False,
    'height_var_k': 250,
    'height_count_threshold': 10,
    'height_var_threshold': 0.1,
    'traffic_light_camera_frame_id': '',
    'traffic_light_camera_channel': '',
    'traffic_light_camera_intrinsics': '',
    'car2stopline_max_distance': 5,
    'car2stopline_max_deg': 45,
}

CALIBRATION_CONF_DIR = 'modules/dreamview_plus_plugins/calibration_tool/conf'
CALIBRATION_CONF_FILE = 'calibration_conf.pb.txt'

DEFAULT_CALIBRATION_PARAMS = {
    'angle': 0.2,
    'distance': 2.0,
    'total': 200,
    'calib_height': True
}


class ToolsTplGenerator:
    """ Tools Template Generator
    """

    def __init__(self, ENV, lidar_list, lidar_params, camera_list):
        self.env = ENV
        self.lidar_list = lidar_list
        for lidar in self.lidar_list:
            if lidar.get('is_main_lidar') is True:
                self.main_lidar = lidar
        self.lidar_params = lidar_params
        self.camera_list = camera_list

    def gen(self):
        """ generate tools conf
        """
        succ, self.tools_conf_tpl = utils.load_yaml_file(self.env, 'tools_conf.tpl.yaml')
        if not succ:
            return False
        if not self.tools_conf_tpl:
            self.tools_conf_tpl = {}
        else:
            if not self.validate_tpl():
                p.error('Validate tools_conf.tpl.yaml failed.')
                return False
        if not self.gen_conf():
            p.error('Generate tools conf failed.')
            return False
        return True

    def validate_tpl(self):
        """ validate tools conf template
        """
        if not isinstance(self.tools_conf_tpl, dict):
            p.error('tools_conf.tpl.yaml formate error: must be a dict.')
            return False
        if 'map_tool_params' in self.tools_conf_tpl:
            if not isinstance(self.tools_conf_tpl['map_tool_params'], dict):
                p.error('tools_conf.tpl.yaml formate error: map_tool_params must be a dict.')
                return False
        if 'calibration_tool_params' in self.tools_conf_tpl:
            if not isinstance(self.tools_conf_tpl['calibration_tool_params'], dict):
                p.error('tools_conf.tpl.yaml formate error: calibration_tool_params must be a dict.')
                return False
        return True

    def update_map_collect_camera_param(self, params):
        """ update map collect camera param
        """

        def update(camera):
            """ update
            """
            params['traffic_light_camera_frame_id'] = camera['frame_id']
            params['traffic_light_camera_channel'] = camera['channel_name']
            params['traffic_light_camera_intrinsics'] = os.path.join(
                self.env['runtime_dir'],
                tf_tpl.INTRINSIC_PARAMS_CONF_DIR,
                tf_tpl.INTRINSIC_PARAMS_CONF_FILE.format(frame_id=camera['frame_id']))
        for camera in self.camera_list:
            if camera.get('used_for_map_collect') is True:
                update(camera)
                return
        for camera in self.camera_list:
            if camera.get('used_for_traffic_light_detection') is True:
                update(camera)
                return

    def gen_conf(self):
        """ generate tools conf
        """
        # generate map conf
        map_tool_conf_dir = os.path.join(self.env['tmp_dir'], IMAGE_CREATOR_CONF_DIR)
        utils.ensure_dir(map_tool_conf_dir)
        params = DEFAULT_IMAGE_CREATOR_PARAMS
        if 'map_tool_params' in self.tools_conf_tpl:
            params.update(**self.tools_conf_tpl['map_tool_params'])
        params['point_cloud_channel'] = self.main_lidar['point_cloud_channel']
        self.update_map_collect_camera_param(params)
        utils.dict_format(params)
        content = tpl.IMAGE_CREATOR_CONF_TPL.format(**params)
        with open(os.path.join(map_tool_conf_dir, IMAGE_CREATOR_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write map tool conf: ' + os.path.join(IMAGE_CREATOR_CONF_DIR, IMAGE_CREATOR_CONF_FILE))
        # generate calibration conf
        calib_tool_conf_dir = os.path.join(self.env['tmp_dir'], CALIBRATION_CONF_DIR)
        utils.ensure_dir(calib_tool_conf_dir)
        params = DEFAULT_CALIBRATION_PARAMS
        if 'calibration_tool_params' in self.tools_conf_tpl:
            params.update(**self.tools_conf_tpl['calibration_tool_params'])
        # lidar
        lidar_content_list = []
        for lidar in self.lidar_list:
            if lidar['frame_id'] not in self.lidar_params:
                p.error(f'lidar {lidar["frame_id"]} not in lidar_params')
                return False
            output_filename = os.path.join(
                self.env['runtime_dir'],
                tf_tpl.EXTRINSIC_PARAMS_CONF_DIR,
                tf_tpl.EXTRINSIC_PARAMS_CONF_FILE.format(parent_frame_id='imu', child_frame_id=lidar['frame_id']))
            lidar_content_list.append(tpl.LIDAR_CALIBRATION_CONF_TPL.format(
                **lidar,
                **utils.get_transform_dict(self.lidar_params[lidar['frame_id']]['transform']),
                output_filename=output_filename))
        params['lidar_list'] = ','.join(lidar_content_list).lstrip()
        # camera
        camera_content_list = []
        for camera in self.camera_list:
            d = dict(camera,  **camera['calibration_params'])
            d['intrinsics_filename'] = os.path.join(
                self.env['runtime_dir'],
                tf_tpl.INTRINSIC_PARAMS_CONF_DIR,
                tf_tpl.INTRINSIC_PARAMS_CONF_FILE.format(frame_id=camera['frame_id']))
            d['output_filename'] = os.path.join(
                self.env['runtime_dir'],
                tf_tpl.EXTRINSIC_PARAMS_CONF_DIR,
                tf_tpl.EXTRINSIC_PARAMS_CONF_FILE.format(child_frame_id=d['frame_id'],
                                                         parent_frame_id=d['lidar_frame_id']))
            d['lidar_extrinsics_filename'] = os.path.join(
                self.env['runtime_dir'],
                tf_tpl.EXTRINSIC_PARAMS_CONF_DIR,
                tf_tpl.EXTRINSIC_PARAMS_CONF_FILE.format(child_frame_id=d['lidar_frame_id'], parent_frame_id='imu'))
            camera_content_list.append(tpl.CAMERA_CALIBRATION_CONF_TPL.format(**d))
        params['camera_list'] = ','.join(camera_content_list).lstrip()
        utils.dict_format(params)
        content = tpl.CALIBRATION_CONF_TPL.format(**params)
        with open(os.path.join(calib_tool_conf_dir, CALIBRATION_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write calibration tool conf: ' + os.path.join(CALIBRATION_CONF_DIR, CALIBRATION_CONF_FILE))
        return True
