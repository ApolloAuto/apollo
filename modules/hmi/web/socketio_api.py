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
"""SocketIO API."""
import google.protobuf.json_format as json_format

from config import Config
from modules.hmi.proto.hmi_message_pb2 import SocketIORequest
import hardware_api
import module_api
import ros_bridge_api
import tool_api

class SocketIOApi(object):
    """
    Socket IO API, which delegate request to target api, and broadcast result
    all HMI frontends.
    """
    API = {
        'hardware_api': hardware_api.HardwareApi,
        'module_api': module_api.ModuleApi,
        'ros_bridge_api': ros_bridge_api.RosBridgeApi,
        'tool_api': tool_api.ToolApi,
    }

    @classmethod
    def execute(cls, socketio_request_json):
        """Execute a command on the given API."""
        Config.log.info('SocketIOApi::execute %s', socketio_request_json)
        socketio_request = json_format.ParseDict(socketio_request_json,
                                                 SocketIORequest())

        api = cls.API.get(socketio_request.api_name)
        if socketio_request.command_name not in dir(api):
            Config.log.critical('Cannot find %s.%s', socketio_request.api_name,
                                socketio_request.command_name)
            return
        cmd = eval('api.' + socketio_request.command_name)
        cmd(*socketio_request.args)
