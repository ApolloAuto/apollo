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
This program can replay a planning output pb file via ros
"""

import argparse
import os.path
import sys

from google.protobuf import text_format
from std_msgs.msg import String
import rospy

import common.message_manager as message_manager


def seq_publisher(seq_num, period):
    """publisher"""

    rospy.init_node('replay_node', anonymous=True)
    messages = {}
    for msg in message_manager.topic_pb_list:
        topic = msg.topic
        name = msg.name
        msg_type = msg.msg_type
        messages[topic] = {}
        filename = str(seq_num) + "_" + name + ".pb.txt"
        print('trying to load pb file: %s' % filename)
        messages[topic]["publisher"] = rospy.Publisher(
            topic, msg_type, queue_size=1)
        pb_msg = msg.parse_file(filename)
        if not pb_msg:
            print('%s pb is none' % topic)
            # continue
        messages[topic]["value"] = pb_msg

    rate = rospy.Rate(int(1.0 / period))  # 10Hz
    while not rospy.is_shutdown():
        for topic in messages:
            if messages[topic]["value"] is not None:
                print('publish: %s' % topic)
                messages[topic]["publisher"].publish(messages[topic]["value"])
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="replay a set of pb files with the same sequence number")
    parser.add_argument(
        "seq",
        action="store",
        type=int,
        default=-1,
        help="set sequence number to replay")
    parser.add_argument(
        "--period",
        action="store",
        type=float,
        default=1,
        help="set the topic publish time duration")
    args = parser.parse_args()
    try:
        seq_publisher(args.seq, args.period)

    except rospy.ROSInterruptException:
        print('Failed to replay message.')
