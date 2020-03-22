#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

""" Sanity Check for calibration table data files. """

import collections
import os
import math
import sys

import colored_glog as glog
import google.protobuf.text_format as text_format

from cyber_py3.record import RecordReader
import modules.common.configs.proto.vehicle_config_pb2 as vehicle_config_pb2

import fueling.common.email_utils as email_utils
import fueling.common.proto_utils as proto_utils
import fueling.common.record_utils as record_utils
import fueling.control.common.multi_vehicle_utils as multi_vehicle_utils

# could be a list
ConfFile = 'vehicle_param.pb.txt'
CHANNELS = {record_utils.CHASSIS_CHANNEL, record_utils.LOCALIZATION_CHANNEL}


def list_records(path):
    glog.info("in list_records:%s" % path)
    records = []
    for (dirpath, _, filenames) in os.walk(path):
        glog.info('filenames: %s' % filenames)
        glog.info('dirpath %s' % dirpath)
        for filename in filenames:
            end_file = os.path.join(dirpath, filename)
            glog.info("end_files: %s" % end_file)
            if record_utils.is_record_file(end_file):
                records.append(end_file)
    return records


def missing_file(path):
    vehicles = multi_vehicle_utils.get_vehicle(path)
    glog.info("vehicles %s" % vehicles)
    for vehicle in vehicles:
        # config file
        conf = os.path.join(path, vehicle, ConfFile)
        glog.info("vehicles conf %s" % conf)
        if os.path.exists(conf) is False:
            glog.error('Missing configuration file in %s' % vehicle)
            return True
        # record file
        glog.info("list of records:" % list_records(os.path.join(path, vehicle)))
        if len(list_records(os.path.join(path, vehicle))) == 0:
            glog.error('No record files in %s' % vehicle)
            return True
    return False


def parse_error(path):
    vehicles = multi_vehicle_utils.get_vehicle(path)
    pb_value = vehicle_config_pb2.VehicleConfig()
    for vehicle in vehicles:
        conf = os.path.join(path, vehicle, ConfFile)
        try:
            proto_utils.get_pb_from_text_file(conf, pb_value)
            return False
        except text_format.ParseError:
            glog.error('Error: Cannot parse %s as binary or text proto' % conf)
            return True


def check_vehicle_id(conf):
    vehicle_id = conf.vehicle_param.vehicle_id
    if vehicle_id.HasField('vin') or vehicle_id.HasField('plate') \
            or vehicle_id.HasField('other_unique_id'):
        return True
    glog.error("Error: No vehicle ID")
    return False


def missing_field(path):
    vehicles = multi_vehicle_utils.get_vehicle(path)
    glog.info("vehicles in missing field: %s" % vehicles)
    for vehicle in vehicles:
        conf_file = os.path.join(path, vehicle, ConfFile)
        glog.info("conf_file: %s" % conf_file)
        # reset for each vehicle to avoid overwrite
        pb_value = vehicle_config_pb2.VehicleConfig()
        conf = proto_utils.get_pb_from_text_file(conf_file, pb_value)
        glog.info("vehicles conf %s" % conf)
        if not check_vehicle_id(conf):
            return True
        # required field
        fields = [conf.vehicle_param.brake_deadzone,
                  conf.vehicle_param.throttle_deadzone,
                  conf.vehicle_param.max_acceleration,
                  conf.vehicle_param.max_deceleration]
        for field in fields:
            if math.isnan(field):
                return True
    return False


def missing_message_data(path, channels=CHANNELS):
    for record in list_records(path):
        glog.info("reading records %s" % record)
        reader = RecordReader(record)
        for channel in channels:
            glog.info("has %d messages" % reader.get_messagenumber(channel))
            if reader.get_messagenumber(channel) == 0:
                return True
    return False


def sanity_check(input_folder, email_receivers=None):
    err_msg = None
    if missing_file(input_folder):
        err_msg = "One or more files are missing in %s" % input_folder
    elif parse_error(input_folder):
        err_msg = "Confige file cannot be parsed in %s" % input_folder
    elif missing_field(input_folder):
        err_msg = "One or more field is missing in Confige file %s" % input_folder
    elif missing_message_data(input_folder):
        err_msg = "Messages are missing in records of %s" % input_folder
    else:
        glog.info("%s Passed sanity check." % input_folder)
        return True

    if email_receivers:
        title = 'Error occurred during data sanity check'
        email_utils.send_email_error(title, {'Error': err_msg}, email_receivers)

    glog.error(err_msg)
    return False


if __name__ == '__main__':
    sanity_check(sys.argv[1])
