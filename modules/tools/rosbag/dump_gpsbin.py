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
This program can extract raw gpsbin from rosbag directory into /tmp/gpsimu.bin.

dump_gpsbin.py rosbag_input_directory output_file

"""

import argparse
import glob
import os
import shutil

from cyber_py3 import cyber
from cyber_py3.record import RecordReader
from modules.drivers.gnss.proto import gnss_pb2


g_args = None
kRawDataTopic = '/apollo/sensor/gnss/raw_data'


def dump_bag(in_dir, out_file):
    """
    out_bag = in_bag
    """
    print('Begin')
    gnss = gnss_pb2.RawData()
    global g_args
    bag_files = glob.glob(in_dir + "/*.record.*")
    with open(out_file, 'w') as fp:
        for bag_file in sorted(bag_files):
            print('Processing bag_file: %s' % bag_file)
            reader = RecordReader(bag_file)
            for msg in reader.read_messages():
                if msg.topic == kRawDataTopic:
                    gnss.ParseFromString(msg.message)
                    f.write(str(gnss))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="A tool to dump gpsimu raw data")
    parser.add_argument(
        "in_dir", action="store", type=str, help="the input bag directory")
    parser.add_argument(
        "out_file", action="store", type=str, help="the output file")

    g_args = parser.parse_args()

    dump_bag(g_args.in_dir, g_args.out_file)
    print("{} is generated".format(g_args.out_file))
