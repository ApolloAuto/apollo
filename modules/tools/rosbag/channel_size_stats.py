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
Collect some channel average size info.

Usage:
    ./channel_size_stats.py <record_path>
        <record_path>    Support * and ?.
Example:
    ./channel_size_stats.py a.record
"""

import argparse
import glob
import os
import sys

import glog

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from cyber.python.cyber_py3.record import RecordWriter
from modules.common_msgs.planning_msgs import planning_pb2


class ChannelSizeStats(object):
    """Sample bags to contain PNC related topics only."""
    TOPICS = [
        '/apollo/canbus/chassis',
        '/apollo/control',
        '/apollo/perception/obstacles',
        '/apollo/perception/traffic_light',
        #        '/apollo/planning',
        '/apollo/prediction',
        '/apollo/localization/pose',
        '/apollo/sensor/camera/front_6mm/image/compressed',
        '/apollo/sensor/lidar128/compensator/PointCloud2'
    ]

    @classmethod
    def process_record(cls, input_record):
        channel_size_stats = {}
        freader = RecordReader(input_record)
        print('----- Begin to process record -----')
        for channelname, msg, datatype, timestamp in freader.read_messages():
            if channelname in ChannelSizeStats.TOPICS:
                if channelname in channel_size_stats:
                    channel_size_stats[channelname]['total'] += len(msg)
                    channel_size_stats[channelname]['num'] += 1
                else:
                    channel_size_stats[channelname] = {}
                    channel_size_stats[channelname]['total'] = len(msg)
                    channel_size_stats[channelname]['num'] = 1.0
            elif channelname == "/apollo/planning":
                adc_trajectory = planning_pb2.ADCTrajectory()
                adc_trajectory.ParseFromString(msg)
                name = "planning_no_debug"
                adc_trajectory.ClearField("debug")
                planning_str = adc_trajectory.SerializeToString()
                if name in channel_size_stats:
                    channel_size_stats[name]['total'] += len(planning_str)
                    channel_size_stats[name]['num'] += 1
                else:
                    channel_size_stats[name] = {}
                    channel_size_stats[name]['total'] = len(planning_str)
                    channel_size_stats[name]['num'] = 1.0

        for channelname in channel_size_stats.keys():
            print(channelname, " num:", channel_size_stats[channelname]['num'],
                  " avg size:", channel_size_stats[channelname]['total'] / channel_size_stats[channelname]['num'])
        print('----- Finish processing record -----')


if __name__ == '__main__':
    cyber.init()
    parser = argparse.ArgumentParser(
        description="Calculate channel average size. \
            Usage: 'python channel_size_stats.py input_record '")
    parser.add_argument('input', type=str, help="the input record")
    args = parser.parse_args()
    ChannelSizeStats.process_record(args.input)
    cyber.shutdown()
