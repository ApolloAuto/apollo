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
Cyber Conf Generator
"""
import os

from modules.conf_gen.diff_gen.file_parser import FileParser
import modules.conf_gen.common.color_print as p


CYBER_CONF = 'cyber/conf/cyber.pb.conf'


class CyberTplGenerator:
    """ Cyber Conf Generator
    """

    def __init__(self, ENV, lidar_list):
        self.env = ENV
        self.lidar_list = lidar_list

    def gen(self):
        """ generate all lidar arena conf
        """
        lidar_params = []
        main_channel_list = []
        other_channel_list = []
        for lidar in self.lidar_list:
            if lidar.get('is_main_lidar'):
                main_channel_list.append(lidar['point_cloud_channel'])
            else:
                other_channel_list.append(lidar['point_cloud_channel'])
        main_channel_list.append('/apollo/sensor/lidar/compensator/PointCloud2')
        for channel in main_channel_list:
            lidar_params.append({'transport_conf.shm_conf.arena_shm_conf.arena_channel_conf[0]': {
                'channel_name': f'"{channel}"',
                'max_msg_size': 33554432,
                'max_pool_size': 32,
            }})
        for channel in other_channel_list:
            lidar_params.append({'transport_conf.shm_conf.arena_shm_conf.arena_channel_conf[0]': {
                'channel_name': f'"{channel}"',
                'max_msg_size': 2097152,
                'max_pool_size': 32,
                'shared_buffer_size': 67108864,
            }})

        params = {'insert': lidar_params}
        file_parser = FileParser(self.env, CYBER_CONF, params)
        if not file_parser.gen():
            p.error(f'Generate cyber conf {CYBER_CONF} failed.')
            return False
        return True
