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
GNSS Template Generator
"""
import os

import modules.conf_gen.tpl_gen.gnss_tpl as tpl
import modules.conf_gen.common.color_print as p

from modules.conf_gen.common import utils

GNSS_CONF_DIR = 'modules/drivers/gnss/conf'
GNSS_CONF_FILE = 'gnss_conf.pb.txt'


class GnssTplGenerator:
    """ Gnss Template Generator
    """

    def __init__(self, ENV):
        self.env = ENV

    def gen(self):
        """ generate all gnss conf
        """
        succ, self.gnss_conf_tpl = utils.load_yaml_file(self.env, 'gnss_conf.tpl.yaml')
        if not succ:
            return False
        if not self.gnss_conf_tpl:
            p.warn('gnss_conf.tpl.yaml not found')
            return True
        if not self.validate_tpl():
            p.error('Validate gnss_conf.tpl.yaml failed.')
            return False
        if not self.gen_conf():
            p.error('Generate gnss conf failed.')
            return False
        return True

    def validate_tpl(self):
        """ validate gnss conf template
        """
        def validate_protocol(data):
            if 'format' not in data:
                p.error('gnss_conf.tpl.yaml format error: format not found.')
                return False
            if data["format"] not in {'NOVATEL_BINARY', 'HUACE_TEXT', 'ASENSING_BINARY', 'BROADGNSS_TEXT'}:
                p.error(f'gnss_conf.tpl.yaml format error: format {data["format"]} is not supported.')
                return False
            if 'protocol' not in data:
                p.error('gnss_conf.tpl.yaml format error: data.protocol not found.')
                return False
            protocol = data['protocol']
            if protocol not in {'serial', 'tcp', 'udp', 'can'}:
                p.error(f'gnss_conf.tpl.yaml format error: protocol {protocol} is not supported.')
                return False
            if 'params' not in data:
                p.error('gnss_conf.tpl.yaml format error: protocol params not found.')
                return False
            if not isinstance(data['params'], dict):
                p.error('gnss_conf.tpl.yaml format error: protocol params must be a dict.')
                return False
            keys = []
            if protocol == 'serial':
                keys = ['device', 'baud_rate']
            elif protocol in ['tcp', 'udp']:
                keys = ['address', 'port']
            elif protocol == 'can':
                keys = ['brand', 'type', 'channel_id', 'interface']
            for key in keys:
                if key not in data['params']:
                    p.error(f'gnss_conf.tpl.yaml format error: protocol params {key} not found.')
                    return False
            return True

        if not isinstance(self.gnss_conf_tpl, dict):
            p.error('gnss_conf.tpl.yaml format error: must be a dict.')
            return False
        # data
        if 'data' not in self.gnss_conf_tpl:
            p.error('gnss_conf.tpl.yaml format error: data not found.')
            return False
        data = self.gnss_conf_tpl['data']
        if not validate_protocol(data):
            return False
        # rtk_from
        if 'rtk_from' in self.gnss_conf_tpl:
            rtk_from = self.gnss_conf_tpl['rtk_from']
            if not isinstance(rtk_from, dict):
                p.error('gnss_conf.tpl.yaml format error: rtk_from must be a dict.')
                return False
        # rtk_to
        if 'rtk_to' in self.gnss_conf_tpl:
            rtk_to = self.gnss_conf_tpl['rtk_to']
            if not isinstance(rtk_to, dict):
                p.error('gnss_conf.tpl.yaml format error: rtk_to must be a dict.')
                return False
            if not validate_protocol(rtk_to):
                return False
        # command
        if 'command' in self.gnss_conf_tpl:
            command = self.gnss_conf_tpl['command']
            if not isinstance(command, dict):
                p.error('gnss_conf.tpl.yaml format error: command must be a dict.')
                return False
            if not validate_protocol(command):
                return False
        return True

    def gen_conf(self):
        """ generate gnss conf
        """
        conf_dir = os.path.join(self.env['tmp_dir'], GNSS_CONF_DIR)
        utils.ensure_dir(conf_dir)

        data_content = tpl.GNSS_DATA_TPL.format(
            format=self.gnss_conf_tpl['data']['format'],
            protocol=self.gen_protocol(self.gnss_conf_tpl['data']))

        rtk_from_content = ''
        if 'rtk_from' in self.gnss_conf_tpl:
            rtk_from_content = tpl.GNSS_RTK_FROM_TPL.format(
                **dict(self.gnss_conf_tpl['rtk_from'], **self.gnss_conf_tpl['rtk_from']['ntrip']))

        rtk_to_content = ''
        if 'rtk_to' in self.gnss_conf_tpl:
            rtk_to_content = tpl.GNSS_RTK_TO_TPL.format(
                format=self.gnss_conf_tpl['rtk_to']['format'],
                protocol=self.gen_protocol(self.gnss_conf_tpl['rtk_to']),
                is_data_stream='true' if self.gnss_conf_tpl['rtk_to'].get('is_data_stream') else 'false')

        command_content = ''
        if 'command' in self.gnss_conf_tpl:
            command_content = tpl.GNSS_COMMAND_TPL.format(
                format=self.gnss_conf_tpl['command']['format'],
                protocol=self.gen_protocol(self.gnss_conf_tpl['command']))

        utils.dict_format(self.gnss_conf_tpl)
        content = tpl.GNSS_CONF_TPL.format(
            data_content=data_content, rtk_from_content=rtk_from_content, rtk_to_content=rtk_to_content,
            command_content=command_content, **self.gnss_conf_tpl)

        with open(os.path.join(conf_dir, GNSS_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write gnss conf: ' + os.path.join(GNSS_CONF_DIR, GNSS_CONF_FILE))
        return True

    def gen_protocol(self, data):
        """ generate protocol conf
        """
        protocol = data['protocol']
        if protocol == 'serial':
            return tpl.GNSS_SERIAL_PROTOCOL_TPL.format(**data['params'])
        elif protocol == 'tcp':
            return tpl.GNSS_TCP_PROTOCOL_TPL.format(**data['params'])
        elif protocol == 'udp':
            return tpl.GNSS_UDP_PROTOCOL_TPL.format(**data['params'])
        elif protocol == 'can':
            return tpl.GNSS_CAN_PROTOCOL_TPL.format(**data['params'])
        else:
            p.error(f'gnss_conf.tpl.yaml format error: unknown protocol {protocol}.')
        return ''
