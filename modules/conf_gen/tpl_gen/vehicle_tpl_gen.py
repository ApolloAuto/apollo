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
Vehicle Template Generator
"""
import os

import modules.conf_gen.tpl_gen.vehicle_tpl as tpl
import modules.conf_gen.common.color_print as p

from modules.conf_gen.common import utils


VEHICLE_PARAMS_CONF_DIR = 'modules/common/data'
VEHICLE_PARAMS_CONF_FILE = 'vehicle_param.pb.txt'
CANBUS_CONF_DIR = 'modules/canbus/conf'
CANBUS_CONF_FILE = 'canbus_conf.pb.txt'
CANBUS_FLAGS_FILE = 'canbus.conf'
CONTROLLERS_CONF_DIR = 'modules/control/control_component/conf'
CONTROLLERS_CONF_FILE = 'pipeline.pb.txt'

VEHICLE_DEFAULT_PARAMS = {
    'min_turn_radius': 2.50,
    'max_acceleration': 2.3,
    'max_deceleration': -7.0,
    'max_steer_angle': 6.2491,
    'max_steer_angle_rate': 6.98131700798,
    'min_steer_angle_rate': 0,
    'steer_ratio': 10.85,
    'wheel_base': 1.73,
    'wheel_rolling_radius': 0.286,
    'max_abs_speed_when_stopped': 0.03,
    'brake_deadzone': 0.1,
    'throttle_deadzone': 0.1,
}
CANBUS_DEFAULT_PARAMS = {
    'max_enable_fail_attempt': 5
}


class VehicleTplGenerator:
    """ Vehicle Template Generator
    """

    def __init__(self, ENV):
        self.env = ENV

    def gen(self):
        """ generate all vehicle conf
        """
        succ, self.vehicle_conf_tpl = utils.load_yaml_file(self.env, 'vehicle_conf.tpl.yaml')
        if not succ:
            return False
        if not self.vehicle_conf_tpl:
            p.warn('vehicle_conf.tpl.yaml not found')
            return True
        if not self.validate_tpl():
            p.error('Validate vehicle_conf.tpl.yaml failed.')
            return False
        if not self.gen_conf():
            p.error('Generate vehicle conf failed.')
            return False
        return True

    def validate_tpl(self):
        """ validate vehicle conf template
        """
        if not isinstance(self.vehicle_conf_tpl, dict):
            p.error('vehicle_conf.tpl.yaml format error: must be a dict.')
            return False
        if 'canbus' not in self.vehicle_conf_tpl:
            p.error('vehicle_conf.tpl.yaml format error: canbus conf not found.')
            return False
        canbus_conf = self.vehicle_conf_tpl['canbus']
        if not isinstance(canbus_conf, dict):
            p.error('vehicle_conf.tpl.yaml format error: canbus conf must be a dict.')
            return False
        for key in ['brand', 'channel_id', 'canbus_vehicle_lib', 'canbus_vehicle_class']:
            if key not in canbus_conf:
                p.error(f'vehicle_conf.tpl.yaml format error: canbus {key} not found.')
                return False

        if 'vehicle_param' not in self.vehicle_conf_tpl:
            p.error('vehicle_conf.tpl.yaml format error: vehicle_param not found.')
            return False
        vehicle_param = self.vehicle_conf_tpl['vehicle_param']
        if not isinstance(vehicle_param, dict):
            p.error('vehicle_conf.tpl.yaml format error: vehicle conf must be a dict.')
            return False
        for key in ['length', 'width', 'height', 'front_edge_to_center', 'back_edge_to_center',
                    'left_edge_to_center', 'right_edge_to_center']:
            if key not in vehicle_param:
                p.error(f'vehicle_conf.tpl.yaml format error: vehicle {key} not found.')
                return False
            if not isinstance(vehicle_param[key], (int, float)):
                p.error(f'vehicle_conf.tpl.yaml format error: vehicle {key} must be a num.')
                return False

        if 'controllers' in self.vehicle_conf_tpl:
            controllers = self.vehicle_conf_tpl['controllers']
            if not isinstance(controllers, list):
                p.error('vehicle_conf.tpl.yaml format error: controllers must be a list.')
                return False
            for controller in controllers:
                if not isinstance(controller, dict):
                    p.error('vehicle_conf.tpl.yaml format error: controller must be a dict.')
                    return False
                for key in ['name', 'type']:
                    if key not in controller:
                        p.error(f'vehicle_conf.tpl.yaml format error: controller {key} not found.')
                        return False
        return True

    def gen_conf(self):
        """ generate vehicle conf
        """
        # generate canbus conf
        canbus_conf_dir = os.path.join(self.env['tmp_dir'], CANBUS_CONF_DIR)
        utils.ensure_dir(canbus_conf_dir)
        canbus_params = dict(CANBUS_DEFAULT_PARAMS, **self.vehicle_conf_tpl['canbus'])
        content = tpl.CANBUS_CONF_TPL.format(**canbus_params)
        with open(os.path.join(canbus_conf_dir, CANBUS_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write canbus conf: ' + os.path.join(CANBUS_CONF_DIR, CANBUS_CONF_FILE))
        content = tpl.CANBUS_FLAGS_TPL.format(**canbus_params)
        with open(os.path.join(canbus_conf_dir, CANBUS_FLAGS_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write canbus flag conf: ' + os.path.join(CANBUS_CONF_DIR, CANBUS_FLAGS_FILE))
        # generate vehicle params conf
        vehicle_conf_dir = os.path.join(self.env['tmp_dir'], VEHICLE_PARAMS_CONF_DIR)
        utils.ensure_dir(vehicle_conf_dir)
        params = dict(VEHICLE_DEFAULT_PARAMS, **self.vehicle_conf_tpl['vehicle_param'], **canbus_params)
        content = tpl.VEHICLE_PARAMS_CONF_TPL.format(**params)
        with open(os.path.join(vehicle_conf_dir, VEHICLE_PARAMS_CONF_FILE), 'w', encoding='utf-8') as f:
            f.write(content.lstrip())
            p.info('write vehicle params conf: ' + os.path.join(VEHICLE_PARAMS_CONF_DIR, VEHICLE_PARAMS_CONF_FILE))
        # generate controller conf
        if 'controllers' in self.vehicle_conf_tpl:
            control_conf_dir = os.path.join(self.env['tmp_dir'], CONTROLLERS_CONF_DIR)
            if not os.path.exists(control_conf_dir):
                os.makedirs(control_conf_dir)
            controllers = self.vehicle_conf_tpl['controllers']
            controller_content = ''
            for controller in controllers:
                controller_content += tpl.CONTROLLER_CONF_TPL.format(**controller)
            with open(os.path.join(control_conf_dir, CONTROLLERS_CONF_FILE), 'w', encoding='utf-8') as f:
                f.write(controller_content.lstrip())
                p.info('write control pipeline conf: ' + os.path.join(CONTROLLERS_CONF_DIR, CONTROLLERS_CONF_FILE))
        return True
