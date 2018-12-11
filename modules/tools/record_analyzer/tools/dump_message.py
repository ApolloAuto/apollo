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
from modules.perception.proto import perception_obstacle_pb2

if __name__ == "__main__":
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
        "-t", "--timestamp", action="store", type=float, required=True,
        help="Specify the timestamp for dumping.")

    args = parser.parse_args()

    record_file = args.file
    bag = rosbag.Bag(record_file, 'r')
    f = file("perception_obstacle.txt", 'w')
    for topic, msg, t in bag.read_messages():
        timestamp = t / float(1e9)
        if (topic == args.message) and abs(timestamp - args.timestamp) <= 1:
            if topic == "/apollo/perception/obstacles":
                f.write(str(msg))
               print str(perception_obstacles)
    f.close()
