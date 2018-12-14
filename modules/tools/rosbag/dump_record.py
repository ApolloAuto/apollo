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
This program can dump a cyber record into separate text files that contains the pb messages
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


def dump_record(in_record, out_dir, start_time, duration, filter_topic):
    freader = record.RecordReader()
    if not freader.open(in_record):
        print("Failed to open: %s" % in_record)
        return
    time.sleep(1)
    seq = 0
    while not freader.endoffile():
        read_msg_succ = freader.read_message()
        if not read_msg_succ:
            print("Read failed")
            return
        t_sec = freader.currentmessage_time()
        if start_time and t_sec < start_time:
            print "not yet reached the start time"
            continue
        if start_time and t_sec >= start_time + duration:
            print "done"
            break
        topic = freader.currentmessage_channelname()
        msg_type = freader.get_messagetype(topic)
        if topic == "/apollo/sensor/mobileye":
            continue
        if not filter_topic or topic == filter_topic:
            message_file = topic.replace("/", "_")
            file_path = os.path.join(out_dir,
                                     str(seq) + message_file + ".pb.txt")
            meta_msg = g_message_manager.get_msg_meta_by_topic(topic)
            if meta_msg is None:
                print("Unknown topic: %s " % topic)
                continue
            msg = meta_msg.msg_type()
            msg.ParseFromString(freader.current_rawmessage())
            write_to_file(file_path, msg)
        seq += 1
    freader.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A tool to dump the protobuf messages in a cyber record into text files"
    )
    parser.add_argument(
        "in_record",
        action="store",
        type=str,
        help="the input cyber record")
    parser.add_argument(
        "--start_time",
        action="store",
        type=float,
        help="the input cyber record")
    parser.add_argument(
        "--duration",
        action="store",
        type=float,
        default=1.0,
        help="the input cyber record")
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
    if not os.path.exists(args.out_dir):
        print("%s does not exist" % args.out_dir)
        sys.exit(0)

    dump_record(args.in_record, args.out_dir, args.start_time, args.duration,
                args.topic)
