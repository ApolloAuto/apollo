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

"""
this module creates a node and fake prediction data based
on json configurations
"""
import argparse
import math
import time

import numpy
import simplejson
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.common_msgs.prediction_msgs.prediction_obstacle_pb2 import PredictionObstacles


def prediction_publisher(prediction_channel, rate):
    """publisher"""
    cyber.init()
    node = cyber.Node("prediction")
    writer = node.create_writer(prediction_channel, PredictionObstacles)
    sleep_time = 1.0 / rate
    seq_num = 1
    while not cyber.is_shutdown():
        prediction = PredictionObstacles()
        prediction.header.sequence_num = seq_num
        prediction.header.timestamp_sec = cyber_time.Time.now().to_sec()
        prediction.header.module_name = "prediction"
        print(str(prediction))
        writer.write(prediction)
        seq_num += 1
        time.sleep(sleep_time)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create empty prediction message",
                                     prog="replay_prediction.py")
    parser.add_argument("-c", "--channel", action="store", type=str, default="/apollo/prediction",
                        help="set the prediction channel")
    parser.add_argument("-r", "--rate", action="store", type=int, default=10,
                        help="set the prediction channel publish time duration")
    args = parser.parse_args()
    prediction_publisher(args.channel, args.rate)
