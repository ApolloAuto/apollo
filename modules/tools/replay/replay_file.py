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
This program can replay a planning output pb file via ros
"""
import os.path
import sys
import argparse
import rospy
import glob
from std_msgs.msg import String
from google.protobuf import text_format

import common.proto_utils as proto_utils
from common.message_manager import PbMessageManager

g_message_manager = PbMessageManager()


def topic_publisher(topic, filename, period):
    """publisher"""
    rospy.init_node('replay_node', anonymous=True)
    meta_msg = None
    msg = None
    if not topic:
        print "Topic not specified, start to guess"
        meta_msg, msg = g_message_manager.parse_file(filename)
        topic = meta_msg.topic()
    else:
        meta_msg = g_message_manager.get_msg_meta_by_topic(topic)
        if not meta_msg:
            print("Failed to find meta info for topic: %s" % (topic))
            return False
        msg = meta_msg.parse_file(filename)
        if not msg:
            print("Failed to parse file[%s] with topic[%s]" % (filename,
                                                               topic))
            return False

    if not msg or not meta_msg:
        print("Unknown topic: %s" % topic)
        return False

    pub = rospy.Publisher(topic, meta_msg.msg_type(), queue_size=1)
    if period == 0:
        while not rospy.is_shutdown():
            raw_input("Press any key to publish one message...")
            pub.publish(msg)
            print("Topic[%s] message published" % topic)
    else:
        rate = rospy.Rate(int(1.0 / period))
        print("started to publish topic[%s] message with rate period %s" %
              (topic, period))
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="replay a planning result pb file")
    parser.add_argument(
        "filename", action="store", type=str, help="planning result files")
    parser.add_argument(
        "--topic", action="store", type=str, help="set the planning topic")
    parser.add_argument(
        "--period",
        action="store",
        type=float,
        default=0.1,
        help="set the topic publish time duration")
    args = parser.parse_args()
    period = 0.0  # use step by step mode
    if args.period:  # play with a given period, (1.0 / frequency)
        period = args.period
    to_replay = args.filename
    files = []
    if os.path.isdir(args.filename):
        files = glob.glob(args.filename + "/*")
        i = 0
        for f in files:
            print "%d  %s" % (i, f)
            i += 1
        str_input = raw_input("Select message by number: ")
        try:
            selected_file = int(str_input)
            if selected_file < 0 or selected_file > len(files):
                print "%d is an invalid number" % selected_file
        except:
            print "%s is not a number" % str_input
        print "Will publish file[%d]: %s" % (selected_file,
                                             files[selected_file])
        to_replay = files[selected_file]
    try:
        topic_publisher(args.topic, to_replay, period)
    except rospy.ROSInterruptException:
        print "failed to replay message"
