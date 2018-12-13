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
from modules.map.proto import map_geometry_pb2

g_message_manager = PbMessageManager()

g_args = None
# mkz vehicle configuration
g_front_to_center = 4.0
g_left_to_center = 1.043 + 0.5
g_right_to_center = 1.043 + 0.5
g_lane_width = 3.7


def create_stop_line(center_x, center_y, heading):
    """create a stop line from center point"""
    left_x = center_x + g_left_to_center * math.cos(heading + math.pi / 2.0)
    left_y = center_y + g_left_to_center * math.sin(heading + math.pi / 2.0)
    right_x = center_x + g_right_to_center * math.cos(heading - math.pi / 2.0)
    right_y = center_y + g_right_to_center * math.sin(heading - math.pi / 2.0)
    stop_line = map_geometry_pb2.Curve()
    curve_segment = stop_line.segment.add()
    left_point = curve_segment.line_segment.point.add()
    left_point.x = left_x
    left_point.y = left_y
    center_point = curve_segment.line_segment.point.add()
    center_point.x = center_x
    center_point.y = center_y
    right_point = curve_segment.line_segment.point.add()
    right_point.x = right_x
    right_point.y = right_y
    return stop_line


def create_signal_proto(x, y, heading):
    # mkz vehicle configuration

    center_x = x + g_front_to_center * math.cos(heading)
    center_y = y + g_front_to_center * math.sin(heading)

    map_signal = map_signal_pb2.Signal()

    map_signal.id.id = "%2.5f_%2.5f" % (center_x, center_y)

    map_signal.type = map_signal_pb2.Signal.MIX_3_VERTICAL

    # left subsignal
    left_subsignal = map_signal.subsignal.add()
    left_x = center_x + g_left_to_center * math.cos(heading + math.pi / 2.0)
    left_y = center_y + g_left_to_center * math.sin(heading + math.pi / 2.0)
    left_subsignal.id.id = "%2.5f_%2.5f" % (left_x, left_y)
    left_subsignal.type = map_signal_pb2.Subsignal.CIRCLE
    left_subsignal.location.x = left_x
    left_subsignal.location.y = left_y
    left_subsignal.location.z = 5.0

    stopline = map_signal.stop_line.add()
    stopline.CopyFrom(create_stop_line(center_x, center_y, heading))

    if g_args.extend_to_neighbor_lane:
        # add stop line on left lane
        left_shift_x = center_x + g_lane_width * math.cos(
            heading + math.pi / 2.0)
        left_shift_y = center_y + g_lane_width * math.sin(
            heading + math.pi / 2.0)
        stopline = map_signal.stop_line.add()
        stopline.CopyFrom(
            create_stop_line(left_shift_x, left_shift_y, heading))

        # add stop line on right lane
        right_shift_x = center_x + g_lane_width * math.cos(
            heading - math.pi / 2.0)
        right_shift_y = center_y + g_lane_width * math.sin(
            heading - math.pi / 2.0)
        stopline = map_signal.stop_line.add()
        stopline.CopyFrom(
            create_stop_line(right_shift_x, right_shift_y, heading))

    return map_signal


def parse_drive_event_file(drive_event_filename, signal_filename):
    drive_event = g_message_manager.parse_topic_file("/apollo/drive_event",
                                                     drive_event_filename)
    if not drive_event:
        print("Failed to find localization in %s" % drive_event_filename)
        return None

    pose = drive_event.location
    map_signal = create_signal_proto(pose.position.x, pose.position.y,
                                     pose.heading)
    proto_utils.write_pb_to_text_file(map_signal, signal_filename)
    return map_signal


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A tool to create traffic light protobuf message from localization.")
    parser.add_argument(
        "drive_event_filename",
        action="store",
        help="""the drive event file name""")
    parser.add_argument(
        "signal_filename", action="store", help="""the signal file name""")
    parser.add_argument(
        "--extend_to_neighbor_lane",
        action="store_true",
        help="""the signal file name""")
    g_args = parser.parse_args()
    parse_drive_event_file(g_args.drive_event_filename, g_args.signal_filename)
