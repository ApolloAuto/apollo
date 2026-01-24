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
Perception Template Generator
"""
import os

import modules.conf_gen.tpl_gen.perception_tpl as tpl
import modules.conf_gen.common.color_print as p

from modules.conf_gen.common import utils

SENSOR_META_CONF_DIR = 'modules/perception/data/conf'
SENSOR_META_CONF_FILE = 'sensor_meta.pb.txt'
POINTCLOUD_PREPROCESS_CONF_DIR = 'modules/perception/pointcloud_preprocess/conf'
POINTCLOUD_PREPROCESS_CONF_FILE = 'pointcloud_preprocess_config.pb.txt'
# POINTCLOUD_PREPROCESS_DAG_DIR = 'modules/perception/pointcloud_preprocess/dag'
# POINTCLOUD_PREPROCESS_DAG_FILE = 'pointcloud_preprocess.dag'
POINRTCLOUD_PREPROCESS_FILTER_DIR = 'modules/perception/pointcloud_preprocess/data'
POINRTCLOUD_PREPROCESS_FILTER_FILE = 'pointcloud_preprocessor.pb.txt'
HDMAP_ROI_FILTER_CONF_DIR = 'modules/perception/pointcloud_map_based_roi/data'
HDMAP_ROI_FILTER_CONF_FILE = 'hdmap_roi_filter.pb.txt'

TRAFFIC_LIGHT_CONF_DIR = 'modules/perception/traffic_light_region_proposal/conf'
TRAFFIC_LIGHT_CONF_FILR = 'traffic_light_region_proposal_config.pb.txt'


DEFAULT_POINTCLOUD_FILTER = {
    'z_threshold': 2.0
}

DEFAULT_HDMAP_ROI_FILTER = {
    'range': 120.0,
    'cell_size': 0.25,
    'extend_dist': 0.0,
    'no_edge_table': False,
    'set_roi_service': True
}


class PerceptionTplGenerator:
    """ Perception Template Generator
    """

    def __init__(self, ENV, main_lidar, camera_list):
        self.env = ENV
        self.main_lidar = main_lidar
        self.camera_list = camera_list

    def gen(self):
        """ generate all perception conf
        """
        succ, self.perception_conf_tpl = utils.load_yaml_file(self.env, 'perception_conf.tpl.yaml')
        if not succ:
            return False
        if not self.perception_conf_tpl:
            p.warn('perception_conf.tpl.yaml not found')
            return True
        if not self.validate_tpl():
            p.error('Validate perception_conf.tpl.yaml failed.')
            return False
        if not self.gen_conf():
            p.error('Generate perception conf failed.')
            return False
        return True

    def validate_tpl(self):
        """ validate perception conf template
        """
        if not isinstance(self.perception_conf_tpl, dict):
            p.error('perception_conf.tpl.yaml format error: must be a dict.')
            return False
        if 'hdmap_roi_filter' not in self.perception_conf_tpl:
            p.error('perception_conf.tpl.yaml format error: hdmap_roi_filter not found.')
            return False
        if not isinstance(self.perception_conf_tpl['hdmap_roi_filter'], dict):
            p.error('perception_conf.tpl.yaml format error: hdmap_roi_filter must be a dict.')
            return False
        # move pointcloud preprocess filter conf into fusion_and_compensator conf
        # if 'pointcloud_filer_zone' not in self.perception_conf_tpl:
        #     p.error('perception_conf.tpl.yaml format error: pointcloud_filer_zone not found.')
        #     return False
        # if not isinstance(self.perception_conf_tpl['pointcloud_filer_zone'], dict):
        #     p.error('perception_conf.tpl.yaml format error: pointcloud_filer_zone must be a dict.')
        #     return False
        # for key in ['box_forward_x', 'box_backward_x', 'box_forward_y', 'box_backward_y']:
        #     if key not in self.perception_conf_tpl['pointcloud_filer_zone']:
        #         p.error(f'perception_conf.tpl.yaml format error: pointcloud_filer_zone.{key} not found.')
        #         return False
        return True

    def gen_conf(self):
        """ generate lidar conf
        """
        conf_dir = os.path.join(self.env['tmp_dir'], SENSOR_META_CONF_DIR)
        utils.ensure_dir(conf_dir)
        # write sensor meta conf
        content = tpl.LIDAR_SENDOR_META_CONF_TPL.format(frame_id=self.main_lidar['frame_id'])
        for camera in self.camera_list:
            content += tpl.CAMERA_SENDOR_META_CONF_TPL.format(**camera)
        with open(os.path.join(conf_dir, SENSOR_META_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write perception sensor meta conf: ' + os.path.join(SENSOR_META_CONF_DIR, SENSOR_META_CONF_FILE))
        # write pointcloud preprocess conf
        conf_dir = os.path.join(self.env['tmp_dir'], POINTCLOUD_PREPROCESS_CONF_DIR)
        utils.ensure_dir(conf_dir)
        content = tpl.POINTCLOUD_PREPROCESS_CONF_TPL.format(frame_id=self.main_lidar['frame_id'])
        with open(os.path.join(conf_dir, POINTCLOUD_PREPROCESS_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write perception pointcloud preprocess conf: ' + os.path.join(POINTCLOUD_PREPROCESS_CONF_DIR,
                   POINTCLOUD_PREPROCESS_CONF_FILE))

        # move pointcloud preprocess filter conf into fusion_and_compensator conf
        # conf_dir = os.path.join(self.env['tmp_dir'], POINRTCLOUD_PREPROCESS_FILTER_DIR)
        # utils.ensure_dir(conf_dir)
        # params = dict(DEFAULT_POINTCLOUD_FILTER, **self.perception_conf_tpl['pointcloud_filer_zone'])
        # utils.dict_format(params)
        # content = tpl.POINRTCLOUD_PREPROCESS_FILTER_TPL.format(**params)
        # with open(os.path.join(conf_dir, POINRTCLOUD_PREPROCESS_FILTER_FILE), 'w', encoding='utf-8') as f:
        #     f.write(content.lstrip())
        #     p.info('write perception pointcloud preprocess filter conf: ' + os.path.join(
        #         POINRTCLOUD_PREPROCESS_FILTER_DIR, POINRTCLOUD_PREPROCESS_FILTER_FILE))

        # write hdmap roi filter conf
        conf_dir = os.path.join(self.env['tmp_dir'], HDMAP_ROI_FILTER_CONF_DIR)
        utils.ensure_dir(conf_dir)
        params = dict(DEFAULT_HDMAP_ROI_FILTER, **self.perception_conf_tpl['hdmap_roi_filter'])
        utils.dict_format(params)
        content = tpl.HDMAP_ROI_FILTER_CONF_TPL.format(**params)
        with open(os.path.join(conf_dir, HDMAP_ROI_FILTER_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write perception hdmap roi filter conf: ' + os.path.join(
                HDMAP_ROI_FILTER_CONF_DIR, HDMAP_ROI_FILTER_CONF_FILE))
        # write traffic light conf
        if self.camera_list:
            conf_dir = os.path.join(self.env['tmp_dir'], TRAFFIC_LIGHT_CONF_DIR)
            utils.ensure_dir(conf_dir)
            camera_names = []
            camera_channel_names = []
            for camera in self.camera_list:
                if camera.get('used_for_traffic_light_detection') is True:
                    camera_names.append(camera['frame_id'])
                    camera_channel_names.append(camera['channel_name'])
            if camera_names:
                content = tpl.TRAFFIC_LIGHT_CONF_TPL.format(camera_names=','.join(camera_names),
                                                            camera_channel_names=','.join(camera_channel_names))
                with open(os.path.join(conf_dir, TRAFFIC_LIGHT_CONF_FILR), 'w', encoding='utf-8') as f:
                    f.write(content.lstrip())
                    p.info('write traffic light conf: ' + os.path.join(TRAFFIC_LIGHT_CONF_DIR, TRAFFIC_LIGHT_CONF_FILR))
        return True
