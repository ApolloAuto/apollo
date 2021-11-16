#!/usr/bin/env python3

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
This program can publish drive event message
"""

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

import argparse
import datetime
import shutil
import time
import os
import sys

from modules.tools.common.message_manager import PbMessageManager
from modules.tools.common import proto_utils

g_message_manager = PbMessageManager()

g_args = None

g_localization = None


def OnReceiveLocalization(localization_msg):
    global g_localization
    g_localization = localization_msg


def main(args):
    drive_event_meta_msg = g_message_manager.get_msg_meta_by_topic(
        args.drive_event_topic)
    if not drive_event_meta_msg:
        print('Unknown drive_event topic name: %s' % args.drive_event_topic)
        sys.exit(1)

    localization_meta_msg = g_message_manager.get_msg_meta_by_topic(
        args.localization_topic)
    if not localization_meta_msg:
        print('Unknown localization topic name: %s' % args.localization_topic)
        sys.exit(1)

    cyber.init()
    node = cyber.Node("derive_event_node")
    node.create_reader(localization_meta_msg.topic,
                       localization_meta_msg.msg_type, OnReceiveLocalization)

    writer = node.create_writer(drive_event_meta_msg.topic,
                                drive_event_meta_msg.msg_type)
    seq_num = 0
    while not cyber.is_shutdown():
        event_type = input(
            "Type in Event Type('d') and press Enter (current time: " +
            str(datetime.datetime.now()) + ")\n>")
        event_type = event_type.strip()
        if len(event_type) != 1 or event_type[0].lower() != 'd':
            continue
        current_time = cyber_time.Time.now().to_sec()
        event_str = None
        while not event_str:
            event_str = input("Type Event:>")
            event_str = event_str.strip()
        event_msg = drive_event_meta_msg.msg_type()
        event_msg.header.timestamp_sec = current_time
        event_msg.header.module_name = 'drive_event'
        seq_num += 1
        event_msg.header.sequence_num = seq_num
        event_msg.header.version = 1
        event_msg.event = event_str
        if g_localization:
            event_msg.location.CopyFrom(g_localization.pose)
        writer.write(event_msg)
        time_str = datetime.datetime.fromtimestamp(current_time).strftime(
            "%Y%m%d%H%M%S")
        filename = os.path.join(args.dir, "%s_drive_event.pb.txt" % time_str)
        proto_utils.write_pb_to_text_file(event_msg, filename)
        print('Logged to rosbag and written to file %s' % filename)
        time.sleep(0.1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="A tool to write events when recording rosbag")
    parser.add_argument(
        "--drive_event_topic",
        action="store",
        default="/apollo/drive_event",
        help="""the drive event topic""")
    parser.add_argument(
        "--localization_topic",
        action="store",
        default="/apollo/localization/pose",
        help="""the drive event topic""")
    parser.add_argument(
        "--dir",
        action="store",
        default="data/bag",
        help="""The log export directory.""")
    g_args = parser.parse_args()
    main(g_args)
