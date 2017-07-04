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

"""Tools restful Api."""

import httplib
import os

import flask_restful
import gflags
import glog

import config
import hardware_api
import module_api
import modules.hmi.proto.runtime_status_pb2 as runtime_status_pb2
import ros_service_api
import runtime_status
import system_cmd


class ToolApi(flask_restful.Resource):
    """Tools in Apollo."""

    def get(self, tool_name):
        """Run tool command and return HTTP response as (content, status)."""
        glog.info("ToolApi receives:" + tool_name)
        if tool_name == 'setup_recording':
            if not runtime_status.RuntimeStatus.are_all_modules_ready():
                module_api.ModuleApi.execute_cmd('all', 'start')
            if not runtime_status.RuntimeStatus.are_all_hardware_ready():
                hardware_api.HardwareApi.execute_cmd('all', 'health_check')
        elif tool_name == 'reset_recording':
            file_to_play = config.Config.get_realpath(gflags.FLAGS.file_to_play)
            if os.path.exists(file_to_play):
                os.rename(file_to_play, file_to_play + '.bak')
            # Also stop player in case user has set it up.
            ToolApi._exec_bash_tool('stop_player')
            runtime_status.RuntimeStatus.get_tools().planning_ready = False
        elif tool_name == 'setup_playing':
            if not runtime_status.RuntimeStatus.are_all_modules_ready():
                module_api.ModuleApi.execute_cmd('all', 'start')
            if not runtime_status.RuntimeStatus.are_all_hardware_ready():
                hardware_api.HardwareApi.execute_cmd('all', 'health_check')
            ToolApi._exec_bash_tool('start_player')
        elif tool_name == 'start_playing':
            # The RESET command will try multiple times to change driving mode
            # to MANUAL.
            ros_service_api.RosServiceApi.execute_cmd('reset')
            ros_service_api.RosServiceApi.execute_cmd('start_auto_driving')
        elif tool_name == 'stop_playing':
            ros_service_api.RosServiceApi.execute_cmd('reset')
            ToolApi._exec_bash_tool('stop_player')
        elif tool_name == 'reset_all':
            module_api.ModuleApi.execute_cmd('all', 'stop')
            runtime_status.RuntimeStatus.reset()
            ToolApi._exec_bash_tool('stop_player')
            ToolApi._exec_bash_tool('stop_recording')
        else:
            ToolApi._exec_bash_tool(tool_name)

        ToolApi._update_runtime_status(tool_name)
        return 'OK', httplib.OK

    @staticmethod
    def _exec_bash_tool(tool_name):
        """Execute bash tool configured in config."""
        tool = config.Config.get_tool(tool_name)
        if tool is None:
            msg = 'Cannot find config for tool {}'.format(tool_name)
            glog.fatal(msg)
            return msg, httplib.BAD_REQUEST

        # Construct the command string by joining all components.
        tool.command[0] = config.Config.get_realpath(tool.command[0])
        cmd_str = ' '.join(tool.command)
        system_cmd.run_in_background(cmd_str, tool.stdout_file,
                                     tool.stderr_file)

    @staticmethod
    def _update_runtime_status(tool_name):
        """Update runtime status."""
        ToolStatus = runtime_status_pb2.ToolStatus

        tools_status = runtime_status.RuntimeStatus.get_tools()
        if tool_name == 'setup_recording':
            tools_status.recording_status = ToolStatus.RECORDING_CHECKING
        elif tool_name == 'start_recording':
            tools_status.recording_status = ToolStatus.RECORDING
            tools_status.playing_status = ToolStatus.PLAYING_NOT_READY
        elif tool_name == 'stop_recording':
            tools_status.recording_status = ToolStatus.RECORDING_FINISHED
            runtime_status.RuntimeStatus.stat_playable_duration()
        elif tool_name == 'reset_recording':
            tools_status.recording_status = ToolStatus.RECORDING_READY_TO_CHECK
            tools_status.playing_status = ToolStatus.PLAYING_NOT_READY
        elif tool_name == 'setup_playing':
            tools_status.playing_status = ToolStatus.PLAYING_CHECKING
        elif tool_name == 'stop_playing':
            tools_status.playing_status = ToolStatus.PLAYING_READY_TO_CHECK
            tools_status.planning_ready = False

        runtime_status.RuntimeStatus.broadcast_status_if_changed()
