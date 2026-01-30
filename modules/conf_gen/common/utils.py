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
""" conf gen utils
"""
import os
import yaml
import subprocess
import shutil
from collections import OrderedDict
import modules.conf_gen.common.color_print as p


def get_conf_file(ENV, file_name):
    """ get conf file, rule: vehicle conf overwride vehicle_type conf
    """
    filename = os.path.join(ENV['vehicle_dir'], file_name)
    if os.path.isfile(filename):
        return filename
    if 'vehicle_type_dir' in ENV:
        filename = os.path.join(ENV['vehicle_type_dir'], file_name)
        if os.path.isfile(filename):
            return filename
    return None


def load_yaml_file(ENV, file_name):
    """ load yaml file, rule: vehicle conf overwride vehicle_type conf
    Returns:
        bool: load success or not
        dict: yaml content
    """
    filename = get_conf_file(ENV, file_name)
    if not filename:
        return True, None
    try:
        return True, yaml.safe_load(open(filename, 'r', encoding='utf-8'))
    except Exception as e:
        p.error(f'load yaml file [{filename}] failed: {e}')
        return False, None


def dict_format(d, list_indent=4):
    """ format dict
    """
    for key, value in d.items():
        if isinstance(value, dict):
            dict_format(value)
        if isinstance(value, bool):
            d[key] = str(value).lower()
        if isinstance(value, list):
            d[key] = ''
            for v in value:
                d[key] += ' ' * list_indent + '"' + v + '",\n'
            d[key] = d[key][:-2]


def ensure_dir(dirname):
    """ ensure dir
    """
    if not os.path.exists(dirname):
        os.makedirs(dirname)


def copy_dir(src, dest):
    """ copy dir in python 3.6
    """
    ensure_dir(dest)
    cmd = f'cp -r {src}/* {dest}/'
    subprocess.run(cmd, shell=True, check=True)


def get_transform_dict(transform):
    """ get trnasform dict
    """
    return {
        'rotation_x': transform['rotation']['x'],
        'rotation_y': transform['rotation']['y'],
        'rotation_z': transform['rotation']['z'],
        'rotation_w': transform['rotation']['w'],
        'translation_x': transform['translation']['x'],
        'translation_y': transform['translation']['y'],
        'translation_z': transform['translation']['z'],
    }
