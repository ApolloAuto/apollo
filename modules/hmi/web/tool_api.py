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
"""Tool API."""

import os

import gflags

from config import Config
from hardware_api import HardwareApi
from module_api import ModuleApi
from modules.hmi.proto.runtime_status_pb2 import ToolStatus
from ros_bridge_api import RosBridgeApi
from runtime_status import RuntimeStatus
import system_cmd


class ToolApi(object):
    """
    Tool SocketIO API:
        setup_recording
        start_recording
        stop_recording
        reset_recording
        setup_playing
        start_playing
        stop_playing
        reset_all
        switch_map
    """
    tools_status = RuntimeStatus.get_tools()

    @classmethod
    def setup_recording(cls):
        """SocketIO Api: setup_recording()"""
        if not RuntimeStatus.are_all_modules_ready():
            ModuleApi.start('all')
        if not RuntimeStatus.are_all_hardware_ready():
            HardwareApi.health_check('all')
        cls.tools_status.recording_status = ToolStatus.RECORDING_CHECKING
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def start_recording(cls):
        """SocketIO Api: start_recording()"""
        cls.__exec_bash_tool('start_recording')
        cls.tools_status.recording_status = ToolStatus.RECORDING
        cls.tools_status.playing_status = ToolStatus.PLAYING_NOT_READY
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def stop_recording(cls):
        """SocketIO Api: stop_recording()"""
        cls.__exec_bash_tool('stop_recording')
        cls.tools_status.recording_status = ToolStatus.RECORDING_FINISHED
        RuntimeStatus.stat_playable_duration()

    @classmethod
    def reset_recording(cls):
        """SocketIO Api: reset_recording()"""
        file_to_play = Config.get_realpath(gflags.FLAGS.file_to_play)
        if os.path.exists(file_to_play):
            os.rename(file_to_play, file_to_play + '.bak')
        # Also stop player in case user has set it up.
        cls.__exec_bash_tool('stop_player')
        cls.tools_status.recording_status = ToolStatus.RECORDING_READY_TO_CHECK
        cls.tools_status.playing_status = ToolStatus.PLAYING_NOT_READY
        cls.tools_status.planning_ready = False
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def setup_playing(cls):
        """SocketIO Api: setup_playing()"""
        if not RuntimeStatus.are_all_modules_ready():
            ModuleApi.start('all')
        if not RuntimeStatus.are_all_hardware_ready():
            HardwareApi.health_check('all')
        cls.__exec_bash_tool('start_player')
        cls.tools_status.playing_status = ToolStatus.PLAYING_CHECKING
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def start_playing(cls):
        """SocketIO Api: start_playing()"""
        RosBridgeApi.change_driving_mode('auto')
        cls.tools_status.playing_status = ToolStatus.PLAYING
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def stop_playing(cls):
        """SocketIO Api: stop_playing()"""
        RosBridgeApi.change_driving_mode('manual')
        cls.__exec_bash_tool('stop_player')
        cls.tools_status.playing_status = ToolStatus.PLAYING_READY_TO_CHECK
        cls.tools_status.planning_ready = False
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def reset_all(cls):
        """SocketIO Api: reset_all()"""
        ModuleApi.stop('all')
        RuntimeStatus.reset()
        cls.__exec_bash_tool('stop_player')
        cls.__exec_bash_tool('stop_recording')
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def switch_map(cls, *args):
        """SocketIO Api: switch_map(map_name)"""
        if len(args) != 1:
            Config.log.critical('ToolApi::switch_map bad args')
            return
        map_name = args[0]
        map_conf = Config.get_map(map_name)
        if map_conf is None:
            Config.log.critical('Cannot find %s map', map_name)
            return
        with open(Config.global_flagfile(), 'a') as fout:
            fout.write('\n--map_dir={}\n'.format(map_conf.map_dir))

        RuntimeStatus.pb_singleton.config.current_map = map_name
        RuntimeStatus.broadcast_status_if_changed()

    @staticmethod
    def __exec_bash_tool(tool_name):
        """Execute bash tool configured in config."""
        tool = Config.get_tool(tool_name)
        if tool is None:
            Config.log.critical('Unknown tool %s', tool_name)
            return
        system_cmd.async_run_command_pb(tool)
