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
Config Generate Tool
"""

import argparse
import os
import sys
import shutil
import tempfile

import modules.conf_gen.common.color_print as p
from modules.conf_gen.common import utils
from modules.conf_gen.tpl_gen.tpl_gen import TplGenerator
from modules.conf_gen.diff_gen.diff_gen import DiffGenerator

WORKROOT = os.getenv('APOLLO_ENV_WORKROOT')
if not WORKROOT:
    print('APOLLO_ENV_WORKROOT not found, exit.')
    sys.exit(1)
os.chdir(WORKROOT)

ENV = {
    'CONF_BASE_DIR': os.path.join(WORKROOT, 'vehicle_conf')
}
ENV['VEHICLE_TYPES_DIR'] = os.path.join(ENV['CONF_BASE_DIR'], 'vehicle_types')
ENV['VEHICLES_DIR'] = os.path.join(ENV['CONF_BASE_DIR'], 'vehicles')


def env_check():
    """ check env
    """
    if not os.path.isdir(ENV['CONF_BASE_DIR']):
        p.error(f'config base dir {ENV["CONF_BASE_DIR"]} not found.')
        return False
    if not os.path.isdir(ENV['VEHICLES_DIR']):
        p.error(f'vehicle dir {ENV["VEHICLE_DIR"]} not found.')
        return False
    for src_profile_dir in ENV['src_profile_dirs']:
        if not os.path.isdir(src_profile_dir):
            p.error(f'source profile dir {src_profile_dir} not found.')
            return False
    if not os.path.isdir(ENV['vehicle_dir']):
        p.error(f'vehicle dir {ENV["vehicle_dir"]} not found.')
        return False
    if 'vehicle_type_dir' in ENV:
        if not os.path.isdir(ENV['vehicle_type_dir']):
            p.error(f'vehicle type dir {ENV["vehicle_type_dir"]} not found.')
            return False
    else:
        p.warn(f'vehicle type is not set.')
    return True


def merge_dir(env):
    """ merge dir
    """
    try:
        utils.copy_dir(env['tmp_dir'], env['target_profile_dir'])
        # copy calibration_table.pb.txt
        calibration_filename = 'calibration_table.pb.txt'
        calibration_table_file = utils.get_conf_file(env, calibration_filename)
        if calibration_table_file:
            target_dir = os.path.join(env['target_profile_dir'], 'modules/control/control_component/conf')
            utils.ensure_dir(target_dir)
            shutil.copy(calibration_table_file, os.path.join(target_dir, calibration_filename))
        # copy other configs
        dirs = []
        if 'vehicle_type_dir' in ENV:
            dirs.append(ENV['vehicle_type_dir'])
        dirs.append(env['vehicle_dir'])
        for d in dirs:
            for item in os.listdir(d):
                path = os.path.join(d, item)
                if os.path.isdir(path):
                    utils.copy_dir(path, os.path.join(env['target_profile_dir'], item))
    except Exception as e:
        p.error(f'Generate profile dir failed: {e}')
        return False
    return True


def gen():
    """ generate all conf
    """
    tpl_generator = TplGenerator(ENV)
    if not tpl_generator.gen():
        return False
    diff_generator = DiffGenerator(ENV)
    if not diff_generator.gen():
        return False
    if not merge_dir(ENV):
        return False
    return True


def main():
    """ Main Method
    """
    product_choices = ['park', 'sweeper']
    parser = argparse.ArgumentParser(description='Conf Generator Tool')
    parser.add_argument('vehicle_id', nargs='?', help='vehicle ID')
    parser.add_argument('-l', '--list', action='store_true', required=False,
                        help='list all vehicles')
    parser.add_argument('-s', '--src_profiles', nargs='+', default=['sample'], help='source profile')
    parser.add_argument('-p', '--product', choices=product_choices,
                        default='park', help=f'product: {product_choices}')
    args = parser.parse_args()

    if args.list:
        for vehicle_id in os.listdir(ENV['VEHICLES_DIR']):
            print(vehicle_id)
        sys.exit(0)
    if not args.vehicle_id:
        p.error('vehicle id is required.')
        sys.exit(1)
    ENV['vehicle_id'] = args.vehicle_id
    ENV['vehicle_dir'] = os.path.join(ENV['VEHICLES_DIR'], args.vehicle_id)
    ENV['product'] = args.product
    ENV['src_profile_dirs'] = [os.path.join(WORKROOT, 'profiles', d) for d in args.src_profiles]
    ENV['target_profile_dir'] = os.path.join(WORKROOT, 'profiles', args.vehicle_id)
    vehicle_type_file = os.path.join(ENV['vehicle_dir'], 'vehicle_type.txt')
    if os.path.isfile(vehicle_type_file):
        for line in open(vehicle_type_file).readlines():
            line = line.strip()
            if line:
                ENV['vehicle_type_dir'] = os.path.join(
                    ENV['VEHICLE_TYPES_DIR'], line)
                break

    if not env_check():
        sys.exit(1)
    ENV['tmp_dir'] = tempfile.mkdtemp()
    for src_profile_dir in ENV['src_profile_dirs']:
        utils.copy_dir(src_profile_dir, ENV['tmp_dir'])
    ENV['runtime_dir'] = os.getenv('APOLLO_RUNTIME_PATH', '/apollo')
    succ = gen()
    shutil.rmtree(ENV['tmp_dir'])
    if not succ:
        p.error('conf gen failed.')
        sys.exit(1)
    p.succ("conf gen succ.")


if __name__ == '__main__':
    main()
