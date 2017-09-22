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
"""HMI ros API."""
import rospy

from config import Config
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.hmi.proto.hmi_message_pb2 import HMICommand


class RosBridgeApi(object):
    """
    HMI ros API:
        change_driving_mode [manual | auto]
    """
    ros_command_pub = None
    ros_command_seq_num = 0

    @classmethod
    def init_ros(cls):
        """Init ros node."""
        rospy.init_node('hmi_ros_bridge', log_level=rospy.DEBUG, anonymous=True)
        cls.ros_command_pub = rospy.Publisher(
            '/apollo/hmi_command', HMICommand, queue_size=1)

    @classmethod
    def change_driving_mode(cls, *args):
        """SocketIO Api: change_driving_mode(target_mode)"""
        if len(args) != 1:
            Config.log.critical('RosBridgeApi::change_driving_mode bad args')
            return
        target_mode = args[0]

        Config.log.info('RosBridgeApi change_driving_mode %s', target_mode)

        cmd_msg = cls.__new_command_message()
        sub_cmd = cmd_msg.change_driving_mode
        if target_mode == 'manual':
            sub_cmd.target_mode = Chassis.COMPLETE_MANUAL
        elif target_mode == 'auto':
            sub_cmd.target_mode = Chassis.COMPLETE_AUTO_DRIVE
            sub_cmd.reset_first = True
        else:
            Config.log.critical('Unknown args: %s', str(args))
            return
        Config.log.debug('Publishing message: %s', str(cmd_msg))
        cls.ros_command_pub.publish(cmd_msg)

    @classmethod
    def __new_command_message(cls):
        """Create a new HMICommand message with header filled."""
        cmd_msg = HMICommand()
        cmd_msg.header.timestamp_sec = rospy.get_time()
        cmd_msg.header.module_name = 'hmi_ros_bridge'
        cls.ros_command_seq_num += 1
        cmd_msg.header.sequence_num = cls.ros_command_seq_num
        return cmd_msg
