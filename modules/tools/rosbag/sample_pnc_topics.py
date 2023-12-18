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
Sample PNC topics. For each /path/to/a.record, will generate
/path/to/pnc_sample/a.record.

Usage:
    ./sample_pnc_topics.py <record_path>
        <record_path>    Support * and ?.
Example:
    ./sample_pnc_topics.py '/mnt/nfs/public_test/2018-04-??/*/mkz8/*/*.record'
"""

import argparse
import glob
import os
import sys

import glog

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from cyber.python.cyber_py3.record import RecordWriter


class SamplePNC(object):
    """Sample bags to contain PNC related topics only."""
    TOPICS = [
        '/apollo/sensor/conti_radar',
        '/apollo/sensor/delphi_esr',
        '/apollo/sensor/gnss/best_pose',
        '/apollo/sensor/gnss/corrected_imu',
        '/apollo/sensor/gnss/gnss_status',
        '/apollo/sensor/gnss/imu',
        '/apollo/sensor/gnss/ins_stat',
        '/apollo/sensor/gnss/odometry',
        '/apollo/sensor/gnss/rtk_eph',
        '/apollo/sensor/gnss/rtk_obs',
        '/apollo/sensor/mobileye',
        '/apollo/canbus/chassis',
        '/apollo/canbus/chassis_detail',
        '/apollo/control',
        '/apollo/control/pad',
        '/apollo/navigation',
        '/apollo/perception/obstacles',
        '/apollo/perception/traffic_light',
        '/apollo/planning',
        '/apollo/prediction',
        '/apollo/localization/pose',
        '/apollo/drive_event',
        '/tf',
        '/tf_static',
        '/apollo/monitor',
        '/apollo/monitor/system_status',
        '/apollo/monitor/static_info',
    ]

    @classmethod
    def process_record(cls, input_record, output_record):
        print("filtering: {} -> {}".format(input_record, output_record))
        output_dir = os.path.dirname(output_record)
        if output_dir != "" and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        freader = RecordReader(input_record)
        fwriter = RecordWriter()
        if not fwriter.open(output_record):
            print('writer open failed!')
            return
        print('----- Begin to process record -----')
        for channelname, msg, datatype, timestamp in freader.read_messages():
            if channelname in SamplePNC.TOPICS:
                desc = freader.get_protodesc(channelname)
                fwriter.write_channel(channelname, datatype, desc)
                fwriter.write_message(channelname, msg, timestamp)
        print('----- Finish processing record -----')


if __name__ == '__main__':
    cyber.init()
    parser = argparse.ArgumentParser(
        description="Filter pnc record. \
            Usage: 'python sample_pnc_topic.py input_record  output_record'")
    parser.add_argument('input', type=str, help="the input record")
    parser.add_argument('output', type=str, help="the output record")
    args = parser.parse_args()
    SamplePNC.process_record(args.input, args.output)
    cyber.shutdown()
