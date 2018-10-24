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

import sys
import argparse
from cyber_py.record import RecordReader
from modules.control.proto import control_cmd_pb2
from modules.planning.proto import planning_pb2
from modules.canbus.proto import chassis_pb2
from modules.drivers.proto import pointcloud_pb2
from module_control_analyzer import ControlAnalyzer
from module_planning_analyzer import PlannigAnalyzer
from lidar_endtoend_analyzer import LidarEndToEndAnalyzer

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "usage: python main.py record_file"
    parser = argparse.ArgumentParser(
        description="Recode Analyzer is a tool to analyze record files.",
        prog="main.py")

    parser.add_argument(
        "-f", "--file", action="store", type=str, required=True,
        help="Specify the record file for message dumping.")

    parser.add_argument(
        "-m", "--message", action="store", type=str, required=True,
        help="Specify the message topic for dumping.")

    parser.add_argument(
        "-t", "--timestamp", action="store", type=long, required=True,
        help="Specify the timestamp fpr dumping.")

    args = parser.parse_args()

    record_file = args.file
    reader = RecordReader(record_file)

    for msg in reader.read_messages():
        print msg.data_type
        print msg.topic
        print msg.timestamp
        if msg.topic == args.message and (msg.timestamp - args.timestamp) < 1:
            print msg.topic
            data = msg.data_type()
            data.ParseFromString(msg.message)
            print data
