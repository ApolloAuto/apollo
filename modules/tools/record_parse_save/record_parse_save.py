#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
function to parse data from *.record files, created using Apollo-Auto

current implementation illustrates sample record file parsing for
* radar (Continental ars-408)
* camera (Leopard Imaging 6mm)
* lidar (Velodyne vls-128)

* saves extracted images in separate folder using *.jpg format
* saves radar and lidar data in respective folders in *.txt format for each scan
* also saves timestamp in separate text files

"""

import os
import sys
import time

from importlib import import_module
import yaml

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import record


os.system('clear')

def read_parameters(yaml_file):
    """
    function to read YAML parameter file and define output destinations
    """
    with open(yaml_file, 'r') as f:
        params = yaml.load(f)
    # record file params
    RECORD_FOLDER = params['records']['filepath']
    parse_type = params['parse']

    # define destinations
    dest_path = os.path.split(RECORD_FOLDER)
    if not dest_path[-1]:
        dest_path = os.path.split(dest_path[0])

    OUT_FOLDER = dest_path[0] + '/'
    temp_path = os.path.split(dest_path[0])
    FOLDER_PREFIX = temp_path[1].replace("-", "")

    parse_dict = {"params": params,
                  "parse_type": parse_type,
                  "out_folder": OUT_FOLDER,
                  "prefix": FOLDER_PREFIX,
                  "record_folder": RECORD_FOLDER}

    return parse_dict

def define_destinations(parse_dict):
    """
    define destination for extracted files
    """
    dest_dict = {
        "channel_name": "",
        "timestamp_file": "",
        "destination_folder": ""
    }

    parse_type = parse_dict["parse_type"]
    params = parse_dict["params"]
    dest_folder = parse_dict["out_folder"]
    prefix = parse_dict["prefix"]

    parser_func = 'parse_' + parse_type

    dest_dict['channel_name'] = params[parse_type]['channel_name']
    dest_dict['timestamp_file'] = dest_folder + prefix + params[parse_type]['timestamp_file_extn']
    dest_dict['destination_folder'] = dest_folder + \
        prefix + params[parse_type]['out_folder_extn'] + '/'

    if not os.path.exists(dest_dict["destination_folder"]):
        os.makedirs(dest_dict["destination_folder"])

    return dest_dict, parser_func

def parse_apollo_record(parse_dict, dest_dict, parser_func):
    """
    """
    record_folder_path = parse_dict["record_folder"]
    parse_type = parse_dict["parse_type"]
    record_files = sorted(os.listdir(parse_dict["record_folder"]))
    parse_timestamp = []
    parse_mod = import_module(parser_func)

    print("=" * 60)
    print('--------- Parsing data for: ' + parse_type + ' ---------')

    for rfile in record_files:
        print("=" * 60)
        print("parsing record file: %s" % rfile)
        freader = record.RecordReader(record_folder_path + rfile)
        time.sleep(.025)

        for channelname, msg, datatype, timestamp in freader.read_messages():
            if channelname == dest_dict["channel_name"]:
                tstamp = parse_mod.parse_data(channelname, msg, dest_dict['destination_folder'])
                parse_timestamp.append(tstamp)

        # write radar-timestamp files
        with open(dest_dict["timestamp_file"], 'w+') as f:
            for item in parse_timestamp:
                f.write("%s\n" % item)

    print("=" * 60)
    print('DONE: records parsed and data saved to: \n  ' + dest_dict['destination_folder'])
    print("=" * 60)

if __name__ == '__main__':
    cyber.init()
    parse_dict = read_parameters('modules/tools/record_parse_save/parser_params.yaml')
    dest_dict, parser_func = define_destinations(parse_dict)
    parse_apollo_record(parse_dict, dest_dict, parser_func)

    cyber.shutdown()
