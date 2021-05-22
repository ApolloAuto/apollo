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
import time

from absl import app
from absl import flags
from absl import logging
from datetime import datetime

from cyber.python.cyber_py3 import cyber
from modules.dreamview.proto import preprocess_table_pb2
from modules.tools.vehicle_calibration.sanity_check import sanity_check

flags.DEFINE_string('vehicle_type', '', 'The vehicle type to be calibrated')
flags.DEFINE_string('data_path', '/apollo/output', 'Default output data path')
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
    cyber.init("Preprocessor")
    preprocessor = Preprocessor()
    task_dir = preprocessor.create_tree()
    preprocessor.sanity_check_path(task_dir)
    cyber.shutdown()


class Preprocessor(object):
    def __init__(self):
        self.record_num = FLAGS.record_num
        self.vehicle_type = self.folder_case(FLAGS.vehicle_type)
        self.config_file = self.get_config_path()
        self.node = cyber.Node("vehicle_calibration_preprocessor")
        self.writer = self.node.create_writer("/apollo/dreamview/progress",
                                              preprocess_table_pb2.Progress,
                                              10)
        self.progress = preprocess_table_pb2.Progress()
        self.progress.percentage = 0.0
        self.progress.log_string = "Press the button to start preprocessing"

    @staticmethod
    def folder_case(str):
        """Convert a string from title case to folder case"""
        return "_".join(str.lower().split(" "))

    def create_if_not_exists(self, path):
        """Create dir if path does not exists"""
        try:
            if not os.path.exists(path):
                os.makedirs(path)
                self.log_and_publish(f'Sucessfully created {path}')
        except OSError:
            self.log_and_publish(f'Failed to create: {path}', 'error')

        return path

    def get_config_path(self):
        """Get the configuration file of the specified vehicle type"""
        return os.path.join(FLAGS.calibration_data_path, self.vehicle_type,
                            FLAGS.config_file_name)

    def get_records_info(self):
        """Get records required for calibration"""
        res = []
        for dir in os.listdir(FLAGS.record_root_path):
            match = re.match(r'(^\d{4}-\d{2}-\d{2})-(\d{2}-\d{2}-\d{2}_s$)',
                             dir)
            if match is not None:
                record_info = {}
                record_info['rel_path'] = match.group()
                record_info['abs_path'] = os.path.join(FLAGS.record_root_path,
                                                       match.group())
                record_info['prefix'] = match.group(1)
                res.append(record_info)
        if len(res) < self.record_num:
            self.log_and_publish(
                f'The number of records in {FLAGS.record_root_path} '
                f'is less than {self.record_num}', 'error')

        res = sorted(res, key=lambda record: record['rel_path'],
                     reverse=True)[:self.record_num]
        return res

    def log_and_publish(self,
                        str,
                        logging_level="info",
                        status=preprocess_table_pb2.Status.UNKNOWN):
        """Publish the str by cyber writer"""
        if logging_level == 'info':
            logging.info(str)
        elif logging_level == 'warn':
            logging.warn(str)
        elif logging_level == 'error':
            logging.error(str)
        elif logging_level == 'fatal':
            logging.fatal(str)
        else:
            logging.info(str)

        self.progress.log_string = str
        self.progress.status = status
        self.writer.write(self.progress)
        time.sleep(0.5)

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
        finished_records = 0
        self.progress.log_string = 'Start preprocessing...'

        for iter in records_info:
            sub_dir = self.create_if_not_exists(
                os.path.join(records_dir, iter['prefix']))
            shutil.copytree(iter['abs_path'],
                            os.path.join(sub_dir, iter['rel_path']))
            finished_records += 1
            self.progress.percentage = (
                finished_records / self.record_num) * 80.0
            self.writer.write(self.progress)

        self.log_and_publish(
            f'The file tree has been successfully created at {task_dir}.')
        return task_dir

    def sanity_check_path(self, path):
        """Sanity check wrapper"""
        result, log_str = sanity_check(path)
        if result is True:
            self.progress.percentage = 100.0
            self.progress.status = preprocess_table_pb2.Status.SUCCESS
        else:
            self.progress.status = preprocess_table_pb2.Status.FAIL
        self.progress.log_string = log_str
        self.writer.write(self.progress)
        time.sleep(0.5)


if __name__ == "__main__":
    app.run(main)
