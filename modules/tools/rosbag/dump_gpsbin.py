#!/usr/bin/env python

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

dump_gpsbin.py rosbag_input_directory

"""

import argparse
import glob
import os
import shutil

import rosbag
import std_msgs
from std_msgs.msg import String

g_args = None


def dump_bag(in_dir):
    """out_bag = in_bag"""
    global g_args
    bag_files = glob.glob(in_dir + "/*.bag")
    f = file("/tmp/gpsimu.bin", 'w')
    for bag_file in sorted(bag_files):
        print "Processing ", bag_file, " ..."
        bag = rosbag.Bag(bag_file, 'r')
        for topic, msg, t in bag.read_messages():
            if topic == "/apollo/sensor/gnss/raw_data":
                f.write(str(msg))
    f.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A tool to dump gpsimu raw data ")
    parser.add_argument(
        "in_dir", action="store", type=str, help="the input ros bag directory")

    g_args = parser.parse_args()

    dump_bag(g_args.in_dir)
    print "/tmp/gpsimu.bin is generated"
