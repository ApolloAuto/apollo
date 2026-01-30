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
Difference Generator
"""
import os
import yaml
import modules.conf_gen.common.color_print as p

from modules.conf_gen.diff_gen.file_parser import FileParser
from modules.conf_gen.common import utils


class DiffGenerator:
    """ Diff Generator
    """

    def __init__(self, ENV):
        self.env = ENV
        self.diff_files = []
        self.diff_params = {}

    def gen(self):
        """ diff gen
        """
        self.get_diff_files()
        if not self.diff_files:
            return True
        if not self.load_diff_params():
            return False
        if not self.validate_diff_params():
            return False
        for file_path, diff_param in self.diff_params.items():
            if not self.gen_diff(file_path, diff_param):
                return False
        return True

    def get_diff_files(self):
        """ get diff files
        """
        diff_file_subfix = '.diff.yaml'
        dirs = []
        if 'vehicle_type_dir' in self.env:
            dirs.append(self.env['vehicle_type_dir'])
        dirs.append(self.env['vehicle_dir'])
        for d in dirs:
            for f in os.listdir(d):
                if f[-len(diff_file_subfix):] == diff_file_subfix:
                    self.diff_files.append(os.path.join(d, f))

    def load_diff_params(self):
        """ load diff params
        """
        for f in self.diff_files:
            try:
                data = yaml.safe_load(open(f, 'r', encoding='utf-8'))
                if isinstance(data, dict):
                    self.diff_params.update(data)
            except Exception as e:
                p.error(f'Failed to load diff file {f}: {e}')
                return False
        return True

    def validate_diff_params(self):
        """ validate diff params
        """
        for k, v in self.diff_params.items():
            if not isinstance(k, str):
                p.error(f'Invalid diff param: {k}')
                return False
            if not os.path.isfile(os.path.join(self.env['tmp_dir'], k)):
                p.error(f'Invalid diff param: file {k} not exists.')
                return False
            if not isinstance(v, dict):
                p.error(f'Invalid diff param: {k}')
                return False
            for k_, items in v.items():
                if k_ not in {'update', 'add', 'delete', 'insert'}:
                    p.error(f'Invalid diff param: {k_} is invalid.')
                    return False
                if not isinstance(items, list):
                    p.error(f'Invalid diff param: {k_} params is invalid.')
                    return False
                for item in items:
                    if k_ == 'delete':
                        if not isinstance(item, str):
                            p.error(f'Invalid diff param: {k} {k_} params is invalid.')
                    elif not isinstance(item, dict):
                        p.error(f'Invalid diff param: {k} {k_} params is invalid.')
                        return False
        return True

    def gen_diff(self, file_path, diff_param):
        """ gen diff
        """
        source_file = os.path.join(self.env['tmp_dir'], file_path)
        file_parser = FileParser(self.env, file_path, diff_param)
        return file_parser.gen()
