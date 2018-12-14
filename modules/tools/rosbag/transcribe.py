#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
This program can transcribe a protobuf message to file
"""

from cyber_py import cyber

import argparse
import shutil
import os
import sys
import time

import common.proto_utils as proto_utils
from common.message_manager import PbMessageManager

g_message_manager = PbMessageManager()
g_args = None


def transcribe(proto_msg):
    header = proto_msg.header
    seq = "%05d" % header.sequence_num
    name = header.module_name
    file_path = os.path.join(g_args.out_dir, seq + "_" + name + ".pb.txt")
    print file_path
    proto_utils.write_pb_to_text_file(proto_msg, file_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A tool to transcribe received protobuf messages into text files")
    parser.add_argument(
        "topic", action="store", help="the topic that you want to transcribe.")
    parser.add_argument(
        "--out_dir",
        action="store",
        default='.',
        help="the output directory for the dumped file, the default value is current directory"
    )
    g_args = parser.parse_args()
    if not os.path.exists(g_args.out_dir):
        os.makedirs(g_args.out_dir)
    meta_msg = g_message_manager.get_msg_meta_by_topic(g_args.topic)
    if not meta_msg:
        print "Unknown topic name: %s" % (g_args.topic)
        sys.exit(0)
    cyber.init()
    node = cyber.Node("transcribe_node")
    node.create_reader(g_args.topic, meta_msg.msg_type, transcribe)
    while not cyber.is_shutdown():
        time.sleep(0.005)
