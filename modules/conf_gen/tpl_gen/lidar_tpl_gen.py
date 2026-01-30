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
Lidar Template Generator
"""
import os
import json

import modules.conf_gen.tpl_gen.lidar_tpl as tpl
import modules.conf_gen.common.color_print as p

from modules.conf_gen.common import utils


LIDAR_CONF_DIR = 'modules/drivers/lidar/conf'
LIDAR_DAG_DIR = 'modules/drivers/lidar/dag'
LIDAR_DAG_FILE = 'lidar.dag'

# RSLIDAR
RSLIDAR_CONF_VARS = {
    'source_type': 'ONLINE_LIDAR',
    'msop_port': 6699,
    'difop_port': 7788,
    'echo_mode': 1,
    'start_angle': 0,
    'end_angle': 360,
    'min_distance': 0.2,
    'max_distance': 200,
    'cut_angle': 0,
    'split_frame_node': 1,
    'num_pkts_split': 0,
    'use_lidar_clock': False,
    'dense_points': False,
    'ts_first_point': False,
    'wait_for_difop': True,
    'config_from_file': False,
    'angle_path': "",
    'split_angle': 0.0,
    'send_raw_packet': True,
}
RSLIDAR_LIB = 'modules/drivers/lidar/rslidar/librslidar_component.so'
RSLIDAR_CLASS_NAME = 'RslidarComponent'

# Livox
LIVOX_CONF_VARS = {
    'source_type': 'ONLINE_LIDAR',
    'model': 'MID360',
    'enable_sdk_console_log': False,
    'integral_time': 0.1,
    'use_lidar_clock': False,
    "host_ip": "192.168.1.2",
    "cmd_data_port": 56101,
    "push_msg_port": 56201,
    "point_data_port": 56301,
    "imu_data_port": 56401,
    "log_data_port": 56501,
}
LIVOX_LIB = 'modules/drivers/lidar/livox/liblivox_component.so'
LIVOX_CLASS_NAME = 'LivoxLidarComponent'
LIVOX_JSON_FILE = 'livox_config.json'

# Vanjee
VANJEE_CONF_VARS = {
    'source_type': 'ONLINE_LIDAR',
    'model': 'vanjee_720_16',
    'connect_type': 1,
    'host_msop_port': 3001,
    'lidar_msop_port': 3333,
    'host_address': '192.168.10.6',
    'lidar_address': '192.168.10.88',
    'publish_mode': 2,
    'start_angle': 0,
    'end_angle': 360,
    'min_distance': 0.3,
    'max_distance': 120,
    'use_lidar_clock': False,
    'dense_points': False,
    'wait_for_difop': True,
    'config_from_file': False,
    'angle_path': "/usr/local/include/vanjee_driver/param/Vanjee_720_16.csv",
}
VANJEE_LIB = 'modules/drivers/lidar/vanjeelidar/libvanjeelidar_component.so'
VANJEE_CLASS_NAME = 'VanjeelidarComponent'

# lidar type dict
LIDAR_PARAMS = {
    'RoboSense': {
        'conf_vars': RSLIDAR_CONF_VARS,
        'lidar_component_lib': RSLIDAR_LIB,
        'lidar_component_name': RSLIDAR_CLASS_NAME,
        'conf_tpl': tpl.RSLIDAR_CONF_TPL,
    },
    'Livox': {
        'conf_vars': LIVOX_CONF_VARS,
        'lidar_component_lib': LIVOX_LIB,
        'lidar_component_name': LIVOX_CLASS_NAME,
        'conf_tpl': tpl.LIVOX_CONF_TPL,
    },
    'Vanjee': {
        'conf_vars': VANJEE_CONF_VARS,
        'lidar_component_lib': VANJEE_LIB,
        'lidar_component_name': VANJEE_CLASS_NAME,
        'conf_tpl': tpl.VANJEE_CONF_TPL,
    },
}

LIDAR_FUSION_AND_COMPENSATOR_CONF_VARS = {
    'max_interval_ms': 50,
    'drop_expired_data': True,
    'wait_time_s': 0.02,
    'transform_query_timeout': 0.02,
    'rotation_compensation': False,
}
COMPENSATOR_CHANNEL = '/apollo/sensor/lidar/compensator/PointCloud2'
COMPENSATOR_CONF_NAME = 'fusion_and_compensator_conf.pb.txt'


class LidarTplGenerator:
    """ Lidar Template Generator
    """

    def __init__(self, ENV):
        self.env = ENV
        self.main_lidar = None
        self.virtual_lidar = None
        self.lidar_list = []

    def gen(self):
        """ generate all lidar conf
        """
        succ, self.lidar_conf_tpl = utils.load_yaml_file(self.env, 'lidar_conf.tpl.yaml')
        if not succ:
            return False
        if not self.lidar_conf_tpl:
            self.lidar_conf_tpl = {}
            p.warn('lidar_conf.tpl.yaml not found')
            return True
        if not self.validate_tpl():
            p.error('Validate lidar_conf.tpl.yaml failed.')
            return False
        self.lidar_list = [lidar for lidar in self.lidar_conf_tpl['lidar_list'] if not lidar.get('is_virtual_lidar')]
        if not self.gen_conf():
            p.error('Generate lidar conf failed.')
            return False
        if not self.gen_dag():
            p.error('Generate lidar dag failed.')
            return False
        return True

    def validate_tpl(self):
        """ validate lidar conf template
        """
        if not isinstance(self.lidar_conf_tpl, dict):
            p.error('lidar_conf.tpl.yaml format error: must be a dict.')
            return False
        if 'lidar_list' not in self.lidar_conf_tpl:
            p.error('lidar_conf.tpl.yaml format error: lidar_list not found.')
            return False
        lidar_list = self.lidar_conf_tpl['lidar_list']
        if not isinstance(lidar_list, list):
            p.error('lidar_conf.tpl.yaml format error: lidar_list must be a list.')
            return False
        main_lidar_count = 0
        for lidar in lidar_list:
            if not isinstance(lidar, dict):
                p.error('lidar_conf.tpl.yaml format error: every lidar must be a dict.')
                return False
            if 'frame_id' not in lidar:
                p.error('lidar_conf.tpl.yaml format error: frame_id not found.')
                return False
            if 'is_virtual_lidar' in lidar:
                if not isinstance(lidar['is_virtual_lidar'], bool):
                    p.error('lidar_conf.tpl.yaml format error: is_virtual_lidar must be a boolean.')
                    return False
                continue
            if 'lidar_type' not in lidar:
                p.error('lidar_conf.tpl.yaml format error: lidar_type not found.')
                return False
            if 'is_main_lidar' in lidar:
                if not isinstance(lidar['is_main_lidar'], bool):
                    p.error('lidar_conf.tpl.yaml format error: is_main_lidar must be a boolean.')
                    return False
                if lidar['is_main_lidar'] is True:
                    main_lidar_count += 1
                    if 'lidar_nscan' not in lidar:
                        p.error('lidar_conf.tpl.yaml format error: main lidar must have lidar_nscan.')
                        return False
                    if not isinstance(lidar['lidar_nscan'], int):
                        p.error('lidar_conf.tpl.yaml format error: lidar_nscan must be a integer.')
                        return False
        if main_lidar_count != 1:
            p.error('lidar_conf.tpl.yaml format error: must have one main lidar.')
            return False
        if 'fusion_and_compensator' not in self.lidar_conf_tpl:
            p.error('lidar_conf.tpl.yaml format error: fusion_and_compensator not found.')
            return False
        fusion_and_compensator = self.lidar_conf_tpl['fusion_and_compensator']
        if not isinstance(fusion_and_compensator, dict):
            p.error('lidar_conf.tpl.yaml format error: fusion_and_compensator must be a dict.')
            return False
        return True

    def get_livox_json(self):
        """ add livox host items
        """
        data = {}
        for lidar in self.lidar_list:
            if lidar['lidar_type'] != 'Livox':
                continue
            lidar_params = dict(LIVOX_CONF_VARS, **lidar['lidar_params'])
            data.setdefault(lidar_params['model'], {
                'lidar_net_info': {
                    'cmd_data_port': 56100,
                    'push_msg_port': 56200,
                    'point_data_port': 56300,
                    'imu_data_port': 56400,
                    'log_data_port': 56500
                },
                'host_net_info': []
            })
            p.error('append')
            data[lidar_params['model']]['host_net_info'].append({
                'lidar_ip': [lidar_params['lidar_ip']],
                'host_ip': lidar_params['host_ip'],
                'cmd_data_port': lidar_params['cmd_data_port'],
                'push_msg_port': lidar_params['push_msg_port'],
                'point_data_port': lidar_params['point_data_port'],
                'imu_data_port': lidar_params['imu_data_port'],
                'log_data_port': lidar_params['log_data_port']
            })
        return data

    def gen_conf(self):
        """ generate lidar conf
        """
        conf_dir = os.path.join(self.env['tmp_dir'], LIDAR_CONF_DIR)
        utils.ensure_dir(conf_dir)
        filter_configs = ''
        self.other_channels = []
        livox_json = None
        for lidar in self.lidar_conf_tpl['lidar_list']:
            if lidar.get('is_virtual_lidar') is True:
                self.virtual_lidar = lidar
                continue
            if lidar.get('is_main_lidar') is True:
                self.main_lidar = lidar
            else:
                self.other_channels.append(lidar['point_cloud_channel'])
            content = ''
            lidar_type = lidar['lidar_type']
            if lidar_type not in LIDAR_PARAMS:
                p.error(f'lidar_type {lidar_type} not supported')
                return False
            params = dict(LIDAR_PARAMS[lidar_type]['conf_vars'], **dict(lidar, **lidar.get('lidar_params', {})))
            if 'write_scan' not in params:
                params['write_scan'] = True
            utils.dict_format(params)
            if lidar_type == 'Livox':
                # livox write config.json
                config_json_file = os.path.join(
                    LIDAR_CONF_DIR, LIVOX_JSON_FILE)
                if livox_json is None:
                    livox_json = self.get_livox_json()
                    with open(os.path.join(self.env['tmp_dir'], config_json_file), 'w', encoding='utf-8') as f:
                        json.dump(livox_json, f, indent=4)
                        p.info('write livox config json: ' + config_json_file)
                params['config_file_path'] = os.path.join(self.env['runtime_dir'], config_json_file)
                # livox custom custom integral
                params['custom_integral'] = ''
                if 'custom_integral_params' in lidar:
                    custom_integral_str = 'custom_integral {\n'
                    for key, value in lidar['custom_integral_params'].items():
                        custom_integral_str += f'  {key}: {value}\n'
                    custom_integral_str += '}'
                    params['custom_integral'] = custom_integral_str

            content = LIDAR_PARAMS[lidar_type]['conf_tpl'].format(**params)
            conf_filename = f'{lidar["frame_id"]}_conf.pb.txt'
            with open(os.path.join(conf_dir, conf_filename), 'w', encoding='utf-8') as f:
                f.write(content.lstrip())
                p.info('write lidar conf: ' + os.path.join(LIDAR_CONF_DIR, conf_filename))
            # gen filter config
            if 'filter_params' in lidar and lidar['filter_params']:
                filter_configs += 'filter_config {\n'
                filter_configs += f'    frame_id: "{lidar["frame_id"]}"\n'
                for key in ['max_x', 'min_x', 'max_y', 'min_y', 'max_z', 'min_z']:
                    if key in lidar['filter_params']:
                        filter_configs += f'    {key}: {lidar["filter_params"][key]}\n'
                filter_configs += '}\n'
        # fusion and compensator
        fusion_and_compensator = LIDAR_FUSION_AND_COMPENSATOR_CONF_VARS
        fusion_and_compensator['input_channels'] = self.other_channels
        fusion_and_compensator['target_frame_id'] = self.main_lidar['frame_id']
        fusion_and_compensator['filter_configs'] = filter_configs
        if self.virtual_lidar:
            fusion_and_compensator['target_frame_id'] = self.virtual_lidar['frame_id']
        utils.dict_format(fusion_and_compensator, list_indent=4)
        with open(os.path.join(conf_dir, COMPENSATOR_CONF_NAME), 'w', encoding='utf-8') as f:
            f.write(tpl.LIDAR_FUSION_AND_COMPENSATOR_CONF_TPL.format(**fusion_and_compensator).lstrip())
            p.info('write lidar fusion_and_compensator conf: ' + os.path.join(LIDAR_CONF_DIR, COMPENSATOR_CONF_NAME))
        return True

    def gen_dag(self):
        """ generate lidar dag
        """
        dag_dir = os.path.join(self.env['tmp_dir'], LIDAR_DAG_DIR)
        utils.ensure_dir(dag_dir)
        params = {}
        for lidar in self.lidar_list:
            lidar_type = lidar['lidar_type']
            params.setdefault(lidar_type, [])
            params[lidar_type].append({
                'config_name': lidar['frame_id'] + "_config",
                'config_path': os.path.join(self.env['runtime_dir'],
                                            LIDAR_CONF_DIR, f'{lidar["frame_id"]}_conf.pb.txt'),
            })

        content = ''
        for lidar_type, lidar_params in params.items():
            components_content = ''
            for param in lidar_params:
                components_content += tpl.LIDAR_COMPONENT_TPL.format(
                    class_name=LIDAR_PARAMS[lidar_type]['lidar_component_name'], **param)
            content += tpl.LIDAR_DRIVER_DAG_TPL.format(components=components_content, **LIDAR_PARAMS[lidar_type])
        content += '\n'

        # fusion and compensator
        filename = os.path.join(self.env['runtime_dir'], LIDAR_CONF_DIR, COMPENSATOR_CONF_NAME)
        content += tpl.LIDAR_FUSION_AND_COMPENSATOR_DAG_TPL.format(
            config_file_path=filename, read_channel=self.main_lidar['point_cloud_channel'])

        # write dag file
        dag_dir = os.path.join(self.env['tmp_dir'], LIDAR_DAG_DIR)
        with open(os.path.join(dag_dir, LIDAR_DAG_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write lidar dag: ' + os.path.join(LIDAR_DAG_DIR, LIDAR_DAG_FILE))
        return True
