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
This is a tool to sanity check the vechicle calibration files
"""
import fnmatch
import math
import os
import google.protobuf.text_format as text_format
from absl import logging

from cyber.python.cyber_py3.record import RecordReader
import modules.common_msgs.config_msgs.vehicle_config_pb2 as vehicle_config_pb2
import modules.tools.common.proto_utils as proto_utils
import modules.tools.common.file_utils as file_utils

# could be a list
ConfFile = 'vehicle_param.pb.txt'
CHASSIS_CHANNEL = '/apollo/canbus/chassis'
LOCALIZATION_CHANNEL = '/apollo/localization/pose'
CHANNELS = {CHASSIS_CHANNEL, LOCALIZATION_CHANNEL}


def is_record_file(path):
    """Naive check if a path is a record."""
    return path.endswith('.record') or fnmatch.fnmatch(path, '*.record.?????')


def get_vehicle(path):
    return [
        subdir for subdir in os.listdir(path)
        if os.path.isdir(os.path.join(path, subdir))
    ]


def missing_input_path(path):
    input_size = file_utils.getInputDirDataSize(path)
    if input_size == 0:
        return True
    return False


def list_records(path):
    logging.info("in list_records:%s" % path)
    records = []
    for (dirpath, _, filenames) in os.walk(path):
        logging.info('filenames: %s' % filenames)
        logging.info('dirpath %s' % dirpath)
        for filename in filenames:
            end_file = os.path.join(dirpath, filename)
            logging.info("end_files: %s" % end_file)
            if is_record_file(end_file):
                records.append(end_file)
    return records


def missing_file(path):
    vehicles = get_vehicle(path)
    logging.info(f'vehicles {vehicles}')
    result = []
    for vehicle in vehicles:
        # config file
        conf = os.path.join(path, vehicle, ConfFile)
        logging.info(f'vehicles conf {conf}')
        if not os.path.exists(conf):
            logging.error(f'Missing configuration file in {vehicle}')
            result.append(ConfFile)
        # record file
        logging.info("list of records:" %
                     list_records(os.path.join(path, vehicle)))
        if len(list_records(os.path.join(path, vehicle))) == 0:
            logging.error(f'No record files in {vehicle}')
            result.append('record')
        if len(result):
            return True, result
    return False, []


def parse_error(path):
    vehicles = get_vehicle(path)
    pb_value = vehicle_config_pb2.VehicleConfig()
    for vehicle in vehicles:
        conf = os.path.join(path, vehicle, ConfFile)
        try:
            proto_utils.get_pb_from_text_file(conf, pb_value)
            return False
        except text_format.ParseError:
            logging.error(
                f'Error: Cannot parse {conf} as binary or text proto')
            return True


def check_vehicle_id(conf):
    # print(conf.HasField('vehicle_id.other_unique_id'))
    vehicle_id = conf.vehicle_param.vehicle_id
    if vehicle_id.vin or vehicle_id.plate or vehicle_id.other_unique_id:
        return True
    logging.error('Error: No vehicle ID')
    return False


def missing_field(path):
    vehicles = get_vehicle(path)
    logging.info(f'vehicles in missing field: {vehicles}')
    result = []
    for vehicle in vehicles:
        conf_file = os.path.join(path, vehicle, ConfFile)
        logging.info(f'conf_file: {conf_file}')
        # reset for each vehicle to avoid overwrited
        pb_value = vehicle_config_pb2.VehicleConfig()
        conf = proto_utils.get_pb_from_text_file(conf_file, pb_value)
        logging.info(f'vehicles conf {conf}')
        if not check_vehicle_id(conf):
            result.append("vehicle_id")
        # required field
        fields = [
            conf.vehicle_param.brake_deadzone,
            conf.vehicle_param.throttle_deadzone,
            conf.vehicle_param.max_acceleration,
            conf.vehicle_param.max_deceleration
        ]
        # for value in conf.vehicle_param:
        # has field is always true since a default value is given
        for field in fields:
            if math.isnan(field):
                result.append(field)
        if len(result):
            return True, result
    return False, result


def missing_message_data(path, channels=CHANNELS):
    result = []
    for record in list_records(path):
        logging.info(f'reading records {record}')
        reader = RecordReader(record)
        for channel in channels:
            logging.info(f'has {reader.get_messagenumber(channel)} messages')
            if reader.get_messagenumber(channel) == 0:
                result.append(record)
    if len(result):
        return True, result
    return False, []


def sanity_check(input_folder):
    err_msg = None
    path_flag = missing_input_path(input_folder)
    field_flag, field_result = missing_field(input_folder)
    channel_flag, channel_result = missing_message_data(input_folder)
    file_flag, file_result = missing_file(input_folder)
    if path_flag:
        err_msg = f'Input path {input_folder} folder structure is wrong'
    if file_flag:
        err_msg = f'One or more files are missing in {input_folder}'
    elif parse_error(input_folder):
        err_msg = f'Config file cannot be parsed in {input_folder}'
    elif field_flag:
        err_msg = f'One or more fields are missing in {input_folder}'
    elif channel_flag:
        err_msg = (
            'Messages are missing in records channels apollo/chassis or '
            f'apollo/localization/pose, records are {channel_result}')
    else:
        info_msg = f'{input_folder} Passed sanity check.'
        logging.info(info_msg)
        return True, info_msg
    logging.error(err_msg)
    return False, err_msg
