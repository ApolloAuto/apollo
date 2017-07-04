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

import flask_restful
import gflags
import glog
import grpc

import modules.hmi.proto.ros_node_pb2 as ros_node_pb2
import modules.hmi.proto.runtime_status_pb2 as runtime_status_pb2
import runtime_status

gflags.DEFINE_string('hmi_ros_node_service', '127.0.0.1:8897',
                     'Address of HMI ros node grpc service.')


class RosServiceApi(flask_restful.Resource):
    """HMI ros node service."""

    def get(self, cmd_name):
        """Run ros command by sending GRPC requests and return HTTP response."""
        return RosServiceApi.execute_cmd(cmd_name)

    @staticmethod
    def execute_cmd(cmd_name):
        """Run ros command by sending GRPC requests and return HTTP response."""
        channel = grpc.insecure_channel(gflags.FLAGS.hmi_ros_node_service)
        stub = ros_node_pb2.HMIRosNodeStub(channel)

        response = None
        status = runtime_status.RuntimeStatus
        if cmd_name == 'reset':
            request = ros_node_pb2.ChangeDrivingModeRequest(
                action=ros_node_pb2.ChangeDrivingModeRequest.RESET_TO_MANUAL)
            response = stub.ChangeDrivingMode(request)

            # Update runtime status.
            tool_status = status.get_tools()
            if tool_status.playing_status != runtime_status_pb2.ToolStatus.PLAYING_NOT_READY:
                tool_status.playing_status = runtime_status_pb2.ToolStatus.PLAYING_READY_TO_CHECK

        elif cmd_name == 'start_auto_driving':
            request = ros_node_pb2.ChangeDrivingModeRequest(
                action=ros_node_pb2.ChangeDrivingModeRequest.START_TO_AUTO)
            response = stub.ChangeDrivingMode(request)

            # Update runtime status.
            status.get_tools(
            ).playing_status = runtime_status_pb2.ToolStatus.PLAYING
        else:
            error_msg = 'RosServiceApi: Unknown command "{}"'.format(cmd_name)
            glog.error(error_msg)
            return error_msg, httplib.BAD_REQUEST

        status.broadcast_status_if_changed()
        glog.info('Processed command "{}", and get response:{}'.format(cmd_name, response))
        return 'OK', httplib.OK
