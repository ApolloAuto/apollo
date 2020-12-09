#!/usr/bin/env python3

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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
This module provides the preprocessing function of vehicle calibration data
"""
import os
import re
import shutil

from absl import app
from absl import flags
from absl import logging
from datetime import datetime

from sanity_check import sanity_check

flags.DEFINE_string('vehicle_type', '', 'The vehicle type to be calibrated')
flags.DEFINE_string('data_path', '/apollo/data', 'Default apollo data path')
flags.DEFINE_string('calibration_data_path',
                    '/apollo/modules/calibration/data',
                    'Default vehicle configuration file directory')
flags.DEFINE_string('config_file_name', 'vehicle_param.pb.txt',
                    'Default vehicle configuration file name')
flags.DEFINE_string('record_root_path', '/apollo/data/bag',
                    'Default record root path')
flags.DEFINE_integer(
    'record_num', 1, 'The number of record folders '
    'required for this calibration task')

FLAGS = flags.FLAGS


def main(argv):
    preprocessor = Preprocessor()
    task_dir = preprocessor.create_tree()
    sanity_check(task_dir)


class Preprocessor(object):
    def __init__(self):
        self.record_num = FLAGS.record_num
        self.vehicle_type = self.folder_case(FLAGS.vehicle_type)
        self.config_file = self.get_config_path()

    @staticmethod
    def folder_case(str):
        """Convert a string from title case to folder case"""
        return "_".join(str.lower().split(" "))

    @staticmethod
    def create_if_not_exists(path):
        """Create dir if path does not exists"""
        try:
            if not os.path.exists(path):
                os.makedirs(path)
                logging.info(f'Sucessfully created {path}')
            else:
                logging.info(f'{path} has been exist')
        except OSError:
            logging.error(f'Failed to create: {path}')

        return path

    def get_config_path(self):
        """Get the configuration file of the specified vehicle type"""
        return os.path.join(FLAGS.calibration_data_path, self.vehicle_type,
                            FLAGS.config_file_name)

    def get_records_info(self):
        """Get records required for calibration"""
        res = []
        for dir in os.listdir(FLAGS.record_root_path):
            match = re.match(r'(^\d{4}-\d{2}-\d{2})-(\d{2}-\d{2}-\d{2}$)', dir)
            if match is not None:
                record_info = {}
                record_info['rel_path'] = match.group()
                record_info['abs_path'] = os.path.join(FLAGS.record_root_path,
                                                       match.group())
                record_info['prefix'] = match.group(1)
                res.append(record_info)
        if len(res) < self.record_num:
            logging.error(f'The number of records in {FLAGS.record_root_path} '
                          f'is less than {self.record_num}')

        res = sorted(res, key=lambda record: record['rel_path'],
                     reverse=True)[:self.record_num]
        return res

    def create_tree(self):
        """Create file tree according to a specific order"""
        task_dir = self.create_if_not_exists(
            os.path.join(FLAGS.data_path,
                         'task' + datetime.now().strftime("-%Y-%m-%d-%H-%M")))
        vehicle_dir = self.create_if_not_exists(
            os.path.join(task_dir, self.vehicle_type))
        records_dir = self.create_if_not_exists(
            os.path.join(vehicle_dir, "Records"))
        shutil.copy(self.config_file, vehicle_dir)
        records_info = self.get_records_info()

        for iter in records_info:
            sub_dir = self.create_if_not_exists(
                os.path.join(records_dir, iter['prefix']))
            shutil.copytree(iter['abs_path'],
                            os.path.join(sub_dir, iter['rel_path']))

        logging.info('The file tree has been successfully created.')
        return task_dir


if __name__ == "__main__":
    app.run(main)
