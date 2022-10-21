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
Extract messages of gps topic from data record file,
and save them into specified binary file

Usage:
    dump_gpsbin.py --input_file=a.record --output_dir=dir

See the gflags for more optional args.
"""

import os
import sys
import time

import gflags
import glog

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import record
from modules.common_msgs.sensor_msgs.gnss_pb2 import RawData


# Requried flags.
gflags.DEFINE_string('input_file', None, 'Input record file path.')

# Optional flags.
gflags.DEFINE_string('output_dir', './', 'Output directory path.')

# Stable flags which rarely change.
gflags.DEFINE_string('gps_raw_data_channel',
                     '/apollo/sensor/gnss/raw_data',
                     'gps raw data channel.')


def process_record_file(args):
    """Read record file and extract the message with specified channels"""
    freader = record.RecordReader(args.input_file)
    glog.info('#processing record file {}'.format(args.input_file))
    time.sleep(1)
    output_file = os.path.join(args.output_dir, 'gpsimu.bin')
    with open(output_file, 'wb') as outfile:
        for channel, message, _type, _timestamp in freader.read_messages():
            if channel == args.gps_raw_data_channel:
                raw_data = RawData()
                raw_data.ParseFromString(message)
                outfile.write(raw_data.data)


def main():
    """Entry point."""
    gflags.FLAGS(sys.argv)
    process_record_file(gflags.FLAGS)
    return


if __name__ == '__main__':
    main()
