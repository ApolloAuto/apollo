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
This program can extract driving trajectory from a record
"""

import argparse
import shutil
import os
import time
import sys

from cyber_py import cyber
from cyber_py import record
from common.message_manager import PbMessageManager

g_message_manager = PbMessageManager()


def write_to_file(file_path, topic_pb):
    """write pb message to file"""
    f = file(file_path, 'w')
    f.write(str(topic_pb))
    f.close()


def extract_record(in_record, output):
    freader = record.RecordReader(in_record)
    print "begin to extract from record {}".format(in_record)
    time.sleep(1)
    seq = 0
    localization_topic = "/apollo/localization/pose"
    meta_msg = g_message_manager.get_msg_meta_by_topic(localization_topic)
    localization_type = meta_msg.msg_type
    for channelname, msg_data, datatype, timestamp in freader.read_messages():
        topic = channelname
        if topic != localization_topic:
            continue
        msg = localization_type()
        msg.ParseFromString(msg_data)
        pose = msg.pose
        output.write("%s %s %s %s %s %s %s %s\n" %
                     (msg.measurement_time, pose.position.x, pose.position.y,
                      pose.position.z, pose.orientation.qx,
                      pose.orientation.qy, pose.orientation.qz,
                      pose.orientation.qw))
    print "finished extracting from record {}".format(in_record)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A tool to dump the localization messages for map team"
                    "Usage: python extract_trajectory.py bag1 bag2 --output output.txt")
    parser.add_argument(
        "in_record",
        action="store",
        nargs='+',
        type=str,
        help="the input cyber record(s)")
    parser.add_argument(
        "--output",
        action="store",
        type=str,
        help="the output file, default is stdout")
    args = parser.parse_args()

    if (args.output):
        out = file(args.output, 'w')
    else:
        out = sys.stdout
    for record_file in args.in_record:
        extract_record(record_file, out)
    out.close()
    cyber.shutdown()
