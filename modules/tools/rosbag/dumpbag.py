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

import rosbag
import std_msgs
import argparse
import shutil
import os
import sys

from std_msgs.msg import String


def write_to_file(file_path, topic_pb):
    """write pb message to file"""
    f = file(file_path, 'w')
    f.write(str(topic_pb))
    f.close()


def dump_bag(in_bag, out_dir, start_time, duration, filter_topic):
    """out_bag = in_bag + routing_bag"""
    bag = rosbag.Bag(in_bag, 'r')
    seq = 0
    for topic, msg, t in bag.read_messages():
        t_sec = t.secs + t.nsecs / 1.0e9;
        if start_time and t_sec < start_time:
            print "not yet reached the start time"
            continue
        if start_time and t_sec >= start_time + duration:
            print "done"
            break
        if topic == "/apollo/sensor/mobileye":
            continue
        if not filter_topic or topic == filter_topic:
            print "export at time ", t
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
        "--start_time", action="store", type=float, help="the input ros bag")
    parser.add_argument(
        "--duration", action="store", type=float, default=1.0, help="the input ros bag")
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
    args = parser.parse_args()

    if os.path.exists(args.out_dir):
        shutil.rmtree(args.out_dir)
    os.makedirs(args.out_dir)
    dump_bag(args.in_rosbag, args.out_dir, args.start_time, args.duration, args.topic)
