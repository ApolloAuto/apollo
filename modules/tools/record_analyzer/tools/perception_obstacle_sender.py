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

import time
import argparse
import google.protobuf.text_format as text_format
from cyber_py3 import cyber
from cyber_py3 import cyber_time
from modules.perception.proto import perception_obstacle_pb2


def update(perception_obstacles):
    """update perception obstacles timestamp"""
    now = cyber_time.Time.now().to_sec()
    perception_obstacles.header.timestamp_sec = now
    perception_obstacles.header.lidar_timestamp = \
        (int(now) - int(0.5)) * int(1e9)

    for perception_obstacle in perception_obstacles.perception_obstacle:
        perception_obstacle.timestamp = now - 0.5
        for measure in perception_obstacle.measurements:
            measure.timestamp = now - 0.5
    return perception_obstacles


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Recode Analyzer is a tool to analyze record files.",
        prog="main.py")

    parser.add_argument(
        "-f", "--file", action="store", type=str, required=True,
        help="Specify the message file for sending.")

    args = parser.parse_args()

    cyber.init()
    node = cyber.Node("perception_obstacle_sender")
    perception_pub = node.create_writer(
        "/apollo/perception/obstacles",
        perception_obstacle_pb2.PerceptionObstacles)

    perception_obstacles = perception_obstacle_pb2.PerceptionObstacles()
    with open(args.file, 'r') as f:
        text_format.Merge(f.read(), perception_obstacles)

    while not cyber.is_shutdown():
        now = cyber_time.Time.now().to_sec()
        perception_obstacles = update(perception_obstacles)
        perception_pub.write(perception_obstacles)
        sleep_time = 0.1 - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)

    cyber.shutdown()
