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
import shutil

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

    @classmethod
    def setup_recording(cls):
        """SocketIO Api: setup_recording()"""
        if not RuntimeStatus.are_record_replay_modules_ready():
            ModuleApi.start('record_replay_required_modules')
        if not RuntimeStatus.are_all_hardware_ready():
            HardwareApi.health_check('all')
        RuntimeStatus.get_tools().recording_status = (
            ToolStatus.RECORDING_CHECKING)
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def start_recording(cls):
        """SocketIO Api: start_recording()"""
        cls.__exec_bash_tool('start_recording')
        tool_status = RuntimeStatus.get_tools()
        tool_status.recording_status = ToolStatus.RECORDING
        tool_status.playing_status = ToolStatus.PLAYING_NOT_READY
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def stop_recording(cls):
        """SocketIO Api: stop_recording()"""
        cls.__exec_bash_tool('stop_recording')
        RuntimeStatus.get_tools().recording_status = (
            ToolStatus.RECORDING_FINISHED)
        RuntimeStatus.stat_playable_duration()

    @classmethod
    def reset_recording(cls):
        """SocketIO Api: reset_recording()"""
        file_to_play = gflags.FLAGS.file_to_play
        if os.path.exists(file_to_play):
            os.rename(file_to_play, file_to_play + '.bak')
        # Also stop player in case user has set it up.
        cls.__exec_bash_tool('stop_player')
        tool_status = RuntimeStatus.get_tools()
        tool_status.recording_status = ToolStatus.RECORDING_READY_TO_CHECK
        tool_status.playing_status = ToolStatus.PLAYING_NOT_READY
        tool_status.planning_ready = False
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def setup_playing(cls):
        """SocketIO Api: setup_playing()"""
        if not RuntimeStatus.are_record_replay_modules_ready():
            ModuleApi.start('record_replay_required_modules')
        if not RuntimeStatus.are_all_hardware_ready():
            HardwareApi.health_check('all')
        cls.__exec_bash_tool('start_player')
        RuntimeStatus.get_tools().playing_status = ToolStatus.PLAYING_CHECKING
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def start_playing(cls):
        """SocketIO Api: start_playing()"""
        RosBridgeApi.change_driving_mode('auto')
        RuntimeStatus.get_tools().playing_status = ToolStatus.PLAYING
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def stop_playing(cls):
        """SocketIO Api: stop_playing()"""
        RosBridgeApi.change_driving_mode('manual')
        cls.__exec_bash_tool('stop_player')
        tool_status = RuntimeStatus.get_tools()
        tool_status.playing_status = ToolStatus.PLAYING_READY_TO_CHECK
        tool_status.planning_ready = False
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
        map_dir = Config.maps.get(map_name)
        if map_dir is None:
            Config.log.critical('Cannot find map %s', map_name)
            return
        with open(Config.get_pb().global_flagfile, 'a') as fout:
            fout.write('\n--map_dir={}\n'.format(map_dir))

        RuntimeStatus.pb_singleton.config.current_map = map_name
        RuntimeStatus.broadcast_status_if_changed()

    @classmethod
    def switch_vehicle(cls, *args):
        """SocketIO Api: switch_vehicle(vehicle_name)"""
        if len(args) != 1:
            Config.log.critical('ToolApi::switch_vehicle bad args')
            return
        vehicle_name = args[0]
        vehicle_path = Config.vehicles.get(vehicle_name)
        if vehicle_path is None:
            Config.log.critical('Cannot find vehicle %s', vehicle_name)
            return

        # Copy vehicle_param_pb.
        conf_pb = Config.get_pb()
        system_cmd.safe_copy(
            os.path.join(vehicle_path, 'vehicle_param.pb.txt'),
            conf_pb.vehicle_param_pb_target_path)
        # Copy calibration_table.
        system_cmd.safe_copy(
            os.path.join(vehicle_path, 'calibration_table.pb.txt'),
            conf_pb.calibration_table_target_path)
        # Copy velodyne_params.
        system_cmd.safe_copy(
            os.path.join(vehicle_path, 'velodyne_params'),
            Config.get_ros_path(conf_pb.velodyne_params_target_path))
        system_cmd.safe_copy(
            os.path.join(vehicle_path, 'start_velodyne.launch'),
            Config.get_ros_path(conf_pb.velodyne_launch_target_path))
        # Copy gnss_conf.
        system_cmd.safe_copy(
            os.path.join(vehicle_path, 'gnss_params/gnss_conf_mkz.txt'),
            Config.get_ros_path(conf_pb.gnss_conf_target_path))
        system_cmd.safe_copy(
            os.path.join(vehicle_path, 'gnss_params/gnss_driver.launch'),
            Config.get_ros_path(conf_pb.gnss_driver_target_path))

        RuntimeStatus.pb_singleton.config.current_vehicle = vehicle_name
        RuntimeStatus.broadcast_status_if_changed()

    @staticmethod
    def __exec_bash_tool(tool_name):
        """Execute bash tool configured in config."""
        tool = Config.get_tool(tool_name)
        if tool is None:
            Config.log.critical('Unknown tool %s', tool_name)
            return
        system_cmd.async_run_command_pb(tool)
