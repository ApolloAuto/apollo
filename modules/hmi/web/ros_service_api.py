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
"""HMI ros node service restful Api."""

import httplib
import time

import flask_restful
import glog
import rospy

from modules.hmi.proto.runtime_status_pb2 import ToolStatus
import modules.control.proto.pad_msg_pb2 as pad_msg_pb2
import runtime_status


class RosServiceApi(flask_restful.Resource):
    """HMI ros node service."""
    pad_msg_pub = None
    pad_msg_seq_num = 0

    @classmethod
    def init_ros(cls):
        """Init ros node."""
        rospy.init_node('hmi_ros_node_service', anonymous=True)
        cls.pad_msg_pub = rospy.Publisher(
            '/apollo/control/pad', pad_msg_pb2.PadMessage, queue_size=1)

    # Restful interface.
    def get(self, cmd_name):
        """Run ros command and return HTTP response."""
        return RosServiceApi.execute_cmd(cmd_name)

    @classmethod
    def execute_cmd(cls, cmd_name):
        """Run ros command and return HTTP response."""
        status = runtime_status.RuntimeStatus
        tool_status = status.get_tools()
        if cmd_name == 'reset':
            cls.perform_driving_action(pad_msg_pb2.RESET)
            # Update runtime status.
            if tool_status.playing_status != ToolStatus.PLAYING_NOT_READY:
                tool_status.playing_status = ToolStatus.PLAYING_READY_TO_CHECK
        elif cmd_name == 'start_auto_driving':
            cls.perform_driving_action(pad_msg_pb2.START)
            # Update runtime status.
            if tool_status.playing_status == ToolStatus.PLAYING_READY_TO_START:
                tool_status.playing_status = ToolStatus.PLAYING
        elif cmd_name == 'restart_auto_driving':
            # Send one RESET before entering AUTO mode, with 0.5s interval.
            interval = 0.5
            cls.perform_driving_action(pad_msg_pb2.RESET)
            time.sleep(interval)
            cls.execute_cmd('start_auto_driving')
        else:
            error_msg = 'RosServiceApi: Unknown command "{}"'.format(cmd_name)
            glog.error(error_msg)
            return error_msg, httplib.BAD_REQUEST

        status.broadcast_status_if_changed()
        glog.info('Processed command "{}"'.format(cmd_name))
        return 'OK', httplib.OK

    @classmethod
    def perform_driving_action(cls, driving_action):
        """Request to perform driving action."""
        pad_msg = pad_msg_pb2.PadMessage()
        pad_msg.header.timestamp_sec = rospy.get_time()
        pad_msg.header.module_name = "hmi_ros_node_service"
        pad_msg.action = driving_action
        cls.pad_msg_seq_num += 1
        pad_msg.header.sequence_num = cls.pad_msg_seq_num

        glog.info('Publishing message: {}'.format(pad_msg))
        cls.pad_msg_pub.publish(pad_msg)
