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
This program can transcribe a protobuf message to file
"""

import rosbag
import std_msgs
import argparse
import shutil
import os
import rospy
import sys

from std_msgs.msg import String

from modules.planning.proto import planning_pb2
from modules.prediction.proto import prediction_obstacle_pb2
from modules.routing.proto import routing_pb2
from modules.perception.proto import perception_obstacle_pb2

g_args = None

topic_msg_dict = {
    "/apollo/planning": planning_pb2.ADCTrajectory,
    "/apollo/prediction": prediction_obstacle_pb2.PredictionObstacles,
    "/apollo/perception": perception_obstacle_pb2.PerceptionObstacles,
    "/apollo/routing_response": routing_pb2.RoutingResponse,
    "/apollo/routing_request": routing_pb2.RoutingRequest,
}


def write_to_file(file_path, topic_pb):
    """write pb message to file"""
    f = file(file_path, 'w')
    f.write(str(topic_pb))
    f.close()


def transcribe(proto_msg):
    header = proto_msg.header
    seq = "%05d" % header.sequence_num
    name = header.module_name
    file_path = os.path.join(g_args.out_dir, seq + "_" + name + ".pb.txt")
    print file_path
    write_to_file(file_path, proto_msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=
        "A tool to transcribe received protobuf messages into text files")
    parser.add_argument(
        "topic",
        action="store",
        help="""the topic that you want to transcribe.""")
    parser.add_argument(
        "--out_dir",
        action="store",
        default='.',
        help=
        "the output directory for the dumped file, the default value is current directory"
    )
    g_args = parser.parse_args()
    if not os.path.exists(g_args.out_dir):
        os.makedirs(g_args.out_dir)
    if g_args.topic not in topic_msg_dict:
        print "Unknown topic name: %s" % (g_args.topic)
        sys.exit(0)
    rospy.init_node('trascribe_node', anonymous=True)
    rospy.Subscriber(g_args.topic, topic_msg_dict[g_args.topic], transcribe)
    rospy.spin()
