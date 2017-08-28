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
This program can dump a rosbag into separate text files that contains the pb messages
"""

import argparse
import os
import shutil

import rosbag
import std_msgs
from std_msgs.msg import String

g_args = None

g_delta_t = 0.5  # 1 second approximate time match region.


def write_to_file(file_path, topic_pb):
    """write pb message to file"""
    f = file(file_path, 'w')
    f.write(str(topic_pb))
    f.close()


def dump_bag(in_bag, out_dir, filter_topic):
    """out_bag = in_bag + routing_bag"""
    bag = rosbag.Bag(in_bag, 'r')
    seq = 0
    global g_args
    for topic, msg, t in bag.read_messages():
        if g_args.time and (t.secs < g_args.time - g_delta_t
                            or t.secs > g_args.time + g_delta_t):
            continue
        if not filter_topic or (filter_topic and topic == filter_topic):
            message_file = topic.replace("/", "_")
            file_path = os.path.join(out_dir,
                                     str(seq) + message_file + ".pb.txt")
            write_to_file(file_path, msg)
        seq += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=
        "A tool to dump the protobuf messages in a ros bag into text files")
    parser.add_argument(
        "in_rosbag", action="store", type=str, help="the input ros bag")
    parser.add_argument(
        "out_dir",
        action="store",
        help="the output directory for the dumped file")
    parser.add_argument(
        "--topic",
        action="store",
        help="""the topic that you want to dump. If this option is not provided,
        the tool will dump all the messages regardless of the message topic."""
    )
    parser.add_argument(
        "--time",
        type=float,
        action="store",
        help="""The time range to extract""")

    g_args = parser.parse_args()

    if os.path.exists(g_args.out_dir):
        shutil.rmtree(g_args.out_dir)
    os.makedirs(g_args.out_dir)
    dump_bag(g_args.in_rosbag, g_args.out_dir, g_args.topic)
