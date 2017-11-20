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

from modules.localization.proto import localization_pb2
from modules.perception.proto import perception_obstacle_pb2
from modules.perception.proto import traffic_light_detection_pb2
from modules.planning.proto import planning_internal_pb2
from modules.planning.proto import planning_pb2
from modules.prediction.proto import prediction_obstacle_pb2
from modules.routing.proto import routing_pb2

topic_msg_dict = {
    "/apollo/planning":
    planning_pb2.ADCTrajectory,
    "/apollo/prediction":
    prediction_obstacle_pb2.PredictionObstacles,
    "/apollo/perception":
    perception_obstacle_pb2.PerceptionObstacles,
    "/apollo/routing_response":
    routing_pb2.RoutingResponse,
    "/apollo/routing_request":
    routing_pb2.RoutingRequest,
    "/apollo/localization/pose":
    localization_pb2.LocalizationEstimate,
    "/apollo/perception/traffic_light":
    traffic_light_detection_pb2.TrafficLightDetection,
}


def generate_message(msg_type, filename):
    """generate message from file"""
    message = msg_type()
    if not os.path.exists(filename):
        return None
    f_handle = file(filename, 'r')
    text_format.Merge(f_handle.read(), message)
    f_handle.close()
    return message


def identify_topic(filename):
    f_handle = file(filename, 'r')
    file_content = f_handle.read()
    for topic, msg_type in topic_msg_dict.items():
        message = msg_type()
        try:
            if text_format.Merge(file_content, message):
                print "identified topic %s" % topic
                f_handle.close()
                return topic
        except text_format.ParseError as e:
            print "Tried %s, failed" % (topic)
            continue
    f_handle.close()
    return None


def topic_publisher(topic, filename, period):
    """publisher"""
    rospy.init_node('replay_node', anonymous=True)
    if not topic:
        print "Topic not specified, start to guess"
        topic = identify_topic(filename)
    if topic not in topic_msg_dict:
        print "Unknown topic:", topic
        sys.exit(0)
    msg_type = topic_msg_dict[topic]
    pub = rospy.Publisher(topic, msg_type, queue_size=1)
    message = generate_message(msg_type, filename)
    if period == 0:
        while not rospy.is_shutdown():
            raw_input("Press any key to publish one message...")
            pub.publish(message)
            print("message published")
    else:
        rate = rospy.Rate(int(1.0 / period))
        print("started to publish message with rate period %s" % period)
        while not rospy.is_shutdown():
            pub.publish(message)
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
