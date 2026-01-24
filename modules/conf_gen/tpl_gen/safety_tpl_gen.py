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
Safety Conf Generator
"""
import os

from modules.conf_gen.diff_gen.file_parser import FileParser
import modules.conf_gen.common.color_print as p


CHANNEL_CHECK_CONF = 'modules/safety_manager/checker/channel_freq_checker/conf/channel_freq_checker_conf.pb.txt'


class SafetyTplGenerator:
    """ Safety Conf Generator
    """

    def __init__(self, ENV, lidar_list):
        self.env = ENV
        self.lidar_list = lidar_list

    def gen(self):
        """ generate all lidar channel check conf
        """
        if not self.gen_channel_check_conf():
            return False
        return True

    def gen_channel_check_conf(self):
        """ generate channel check conf
        """
        lidar_params = []
        for lidar in self.lidar_list:
            item = {
                'name': f'"{lidar["point_cloud_channel"]}"',
                'level': 'ERROR',
                'expect_freq': 10,
                'lower_limit': 6,
                'upper_limit': 15,
            }
            if 'safety_code' in lidar:
                item['code'] = lidar["safety_code"]
            lidar_params.append({'item[0]': item})
        params = {'insert': lidar_params}
        file_parser = FileParser(self.env, CHANNEL_CHECK_CONF, params)
        if not file_parser.gen():
            p.error(
                f'Generate safety channel freq check conf {CHANNEL_CHECK_CONF} failed.')
            return False
        return True
