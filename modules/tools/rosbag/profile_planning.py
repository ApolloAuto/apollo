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
import rospy
import std_msgs
import argparse
import shutil
import StringIO
import os
import sys

from std_msgs.msg import String
from common.message_manager import PbMessageManager

g_message_manager = PbMessageManager()
g_args = None


def stat_planning(planning_msg):
    stats = {}
    if planning_msg.HasField("estop"):
        return stats
    stats["total_time"] = planning_msg.latency_stats.total_time_ms
    stats["init_time"] = planning_msg.latency_stats.init_frame_time_ms
    used_time = stats["init_time"]
    stats["obstacles"] = len(planning_msg.decision.object_decision.decision)
    for task in planning_msg.latency_stats.task_stats:
        stats[task.name] = task.time_ms
        used_time += task.time_ms
    stats["other"] = stats["total_time"] - used_time
    return stats


g_first_time = True


def print_stat(msg, fhandle):
    if not msg:
        return
    global g_first_time
    keywords = [
        'obstacles', 'total_time', 'init_time', u'TrafficDecider',
        u'DpPolyPathOptimizer', u'PathDecider', u'DpStSpeedOptimizer',
        u'SpeedDecider', u'QpSplinePathOptimizer', u'QpSplineStSpeedOptimizer',
        u'ReferenceLineProvider', u'other'
    ]

    output = StringIO.StringIO()
    valid = True
    if g_first_time:
        g_first_time = False
        output.write("\t".join(keywords) + "\n")
    for key in keywords:
        if key not in msg:
            valid = False
            break
        if key == "obstacles":
            output.write("%d\t" % msg[key])
        else:
            output.write("%.3f\t" % msg[key])
    if valid:
        output.write("\n")
        fhandle.write(output.getvalue())
        fhandle.flush()
    output.close()


def on_receive_planning(planning_msg):
    ss = stat_planning(planning_msg)
    print_stat(ss, g_args.report)


def dump_bag(in_bag, msg_meta, fhandle):
    bag = rosbag.Bag(in_bag, 'r')
    for topic, msg, t in bag.read_messages():
        if topic != msg_meta.topic:
            continue
        ss = stat_planning(msg)
        print_stat(ss, g_args.report)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A tool to profile the runtime statistic of planning module"
    )
    parser.add_argument(
        "--rosbag", action="store", type=str, help="the input ros bag")
    parser.add_argument(
        "--report",
        type=str,
        action="store",
        help="The output file, if not use, will use stdout")
    parser.add_argument(
        "--topic",
        action="store",
        default="/apollo/planning",
        help="The planning module topic name")

    g_args = parser.parse_args()
    if not g_args.report:
        g_args.report = sys.stdout
    else:
        g_args.report = file(g_args.report, 'w')
    meta_msg = g_message_manager.get_msg_meta_by_topic(g_args.topic)
    if not meta_msg:
        print("Failed to find topic[%s] in message manager" % (g_args.topic))
        sys.exit(0)

    if g_args.rosbag:
        dump_bag(g_args.rosbag, meta_msg, g_args.report)
    else:
        rospy.init_node("stat_planning", anonymous=True)
        rospy.Subscriber(g_args.topic, meta_msg.msg_type,
                         on_receive_planning)
        rospy.spin()

    if g_args.report != sys.stdout:
        g_args.report.close()
