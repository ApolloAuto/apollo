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
This program can publish drive event message
"""

import rosbag
import std_msgs
import argparse
import shutil
import os
import rospy
import sys

from std_msgs.msg import String

from common.message_manager import PbMessageManager

g_message_manager = PbMessageManager()

g_args = None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A tool to write events when recording rosbag")
    parser.add_argument(
        "--topic",
        action="store",
        default="/apollo/drive_event",
        help="""the drive event topic""")
    g_args = parser.parse_args()

    meta_msg = g_message_manager.get_msg_meta_by_topic(g_args.topic)
    if not meta_msg:
        print "Unknown topic name: %s" % (g_args.topic)
        sys.exit(0)

    rospy.init_node('drive_event_node', anonymous=True)
    pub = rospy.Publisher(meta_msg.topic(), meta_msg.msg_type(), queue_size=10)

    seq_num = 1

    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()
        event_str = raw_input("Type in event and press Enter (current time: " + str(current_time) + ")\n>")
        event_str = event_str.strip()
        seconds = current_time.secs + current_time.nsecs / 1000.0
        event_msg = meta_msg.msg_type()()
        print event_msg.header
        event_msg.header.timestamp_sec = seconds
        event_msg.header.module_name = "drive_event"
        event_msg.header.sequence_num = seq_num
        seq_num += 1
        event_msg.header.version = 1
        event_msg.event = event_str
        pub.publish(event_msg)
