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
This program can create a traffic light protobuf from localization message
"""

import rosbag
import std_msgs
import argparse
import shutil
import os
import rospy
import sys
import math

from std_msgs.msg import String

import common.proto_utils as proto_utils
from common.message_manager import PbMessageManager

from modules.map.proto import map_signal_pb2

g_message_manager = PbMessageManager()

g_args = None


def parse_drive_event_file(filename):
    drive_event = g_message_manager.parse_topic_file("/apollo/drive_event",
                                                      filename)
    if not drive_event:
        print("Failed to find localization in %s" % filename)
        return None

    # mkz vehicle configuration
    front_to_center = 4.0
    left_to_center = 1.043 + 0.5
    right_to_center = 1.043 + 0.5

    pose = drive_event.location
    x = pose.position.x
    y = pose.position.y
    heading = pose.heading

    stop_x = x + front_to_center * math.cos(heading)
    stop_y = y + front_to_center * math.sin(heading)

    left_x = stop_x + left_to_center * math.cos(heading - math.pi / 2.0)
    left_y = stop_y + left_to_center * math.sin(heading - math.pi / 2.0)
    right_x = stop_x + right_to_center * math.cos(heading + math.pi / 2.0)
    right_y = stop_y + right_to_center * math.sin(heading + math.pi / 2.0)

    map_signal = map_signal_pb2.Signal()

    map_signal.id.id = "%2.5f_%2.5f" % (x, y)

    map_signal.type = map_signal_pb2.Signal.MIX_3_VERTICAL

    # left subsignal
    left_subsignal = map_signal.subsignal.add()
    left_subsignal.id.id = "%2.5f_%2.5f" % (left_x, left_y)
    left_subsignal.type = map_signal_pb2.Subsignal.CIRCLE
    left_subsignal.location.x = left_x
    left_subsignal.location.y = left_y
    left_subsignal.location.z = 5.0

    # right subsignal
    right_subsignal = map_signal.subsignal.add()
    right_subsignal.id.id = "%2.5f_%2.5f" % (right_x, right_y)
    right_subsignal.type = map_signal_pb2.Subsignal.CIRCLE
    right_subsignal.location.x = right_x
    right_subsignal.location.y = right_y
    right_subsignal.location.z = 5.0

    proto_utils.write_pb_to_text_file(map_signal, filename + "_signal.pb.txt")
    return map_signal


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=
        "A tool to create traffic light protobuf message from localization.")
    parser.add_argument(
        "filename", action="store", help="""the localization file name""")
    g_args = parser.parse_args()
    parse_drive_event_file(g_args.filename)
