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

from modules.localization.proto import localization_pb2
from modules.perception.proto import perception_obstacle_pb2
from modules.perception.proto import traffic_light_detection_pb2
from modules.planning.proto import planning_internal_pb2
from modules.planning.proto import planning_pb2
from modules.prediction.proto import prediction_obstacle_pb2
from modules.routing.proto import routing_pb2
from modules.control.proto import control_cmd_pb2
from modules.canbus.proto import chassis_pb2
from modules.common.proto import drive_event_pb2
from modules.map.relative_map.proto import navigation_pb2

import proto_utils


class MessageType:
    def __init__(self, name, topic, msg_type):
        self.__name = name
        self.__topic = topic
        self.__msg_type = msg_type

    def name(self):
        return self.__name

    def topic(self):
        return self.__topic

    def msg_type(self):
        return self.__msg_type

    def instance(self):
        return self.__msg_type()

    def parse_file(self, filename):
        value = self.instance()
        if not proto_utils.get_pb_from_file(filename, value):
            print("Failed to parse file %s" % filename)
            return None
        else:
            return value


topic_pb_list = [
    MessageType("planning", "/apollo/planning", planning_pb2.ADCTrajectory),
    MessageType("control", "/apollo/control", control_cmd_pb2.ControlCommand),
    MessageType("chassis", "/apollo/canbus/chassis", chassis_pb2.Chassis),
    MessageType("prediction", "/apollo/prediction",
                prediction_obstacle_pb2.PredictionObstacles),
    MessageType("perception", "/apollo/perception",
                perception_obstacle_pb2.PerceptionObstacles),
    MessageType("routing_response", "/apollo/routing_response",
                routing_pb2.RoutingResponse),
    MessageType("routing_request", "/apollo/routing_request",
                routing_pb2.RoutingRequest),
    MessageType("localization", "/apollo/localization/pose",
                localization_pb2.LocalizationEstimate),
    MessageType("traffic_light", "/apollo/perception/traffic_light",
                traffic_light_detection_pb2.TrafficLightDetection),
    MessageType("drive_event", "/apollo/drive_event",
                drive_event_pb2.DriveEvent),
    MessageType("relative_map", "/apollo/relative_map", navigation_pb2.MapMsg),
    MessageType("relative_map", "/apollo/navigation",
                navigation_pb2.NavigationInfo),
]


class PbMessageManager:
    def __init__(self):
        self.__topic_dict = {}
        self.__name_dict = {}

        for msg in topic_pb_list:
            self.__topic_dict[msg.topic()] = msg
            self.__name_dict[msg.name()] = msg

    def topic_dict(self):
        return self.__topic_dict

    def get_msg_meta_by_topic(self, topic):
        if topic in self.__topic_dict:
            return self.__topic_dict[topic]
        else:
            return None

    def get_msg_meta_by_name(self, name):
        if name in self.__name_dict:
            return self.__name_dict[name]
        else:
            return None

    def name_dict(self):
        return self.__name_dict

    def parse_topic_file(self, topic, filename):
        if topic not in self.__topic_dict:
            print("topic %s is not registered in topic_pb_list" % topic)
            return None
        meta_msg = self.__topic_dict[topic]
        return meta_msg.parse_file(filename)

    def parse_file(self, filename):
        """parse a file by guessing topic type"""
        for topic, meta_msg in self.__topic_dict.items():
            try:
                message = meta_msg.parse_file(filename)
                if message:
                    print "identified topic %s" % topic
                    return (meta_msg, message)
            except text_format.ParseError as e:
                print "Tried %s, failed" % (topic)
                continue
        return (None, None)
