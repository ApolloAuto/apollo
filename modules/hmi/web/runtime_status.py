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

"""Global runtime status."""

import os
import time

import flask_socketio
import gflags
import glog
import google.protobuf.json_format as json_format

import config
import modules.hmi.proto.runtime_status_pb2 as runtime_status_pb2

gflags.DEFINE_string('file_to_play', 'data/log/garage.csv',
                     'File to check existence to determine player status.')


class RuntimeStatus(object):
    """Global runtime status."""

    pb_singleton = runtime_status_pb2.RuntimeStatus()
    pb_fingerprint = 0

    module_dict = {}
    hardware_dict = {}
    playable_duration = 0

    @classmethod
    def reset(cls, check_playable_file=False):
        """Reset runtime status to start."""
        cls.pb_singleton.Clear()
        cls.pb_fingerprint = 0
        cls.module_dict.clear()
        cls.hardware_dict.clear()

        tool_status = cls.get_tools()
        if check_playable_file and cls.stat_playable_duration() > 0:
            tool_status.recording_status = runtime_status_pb2.ToolStatus.RECORDING_FINISHED
            tool_status.playing_status = runtime_status_pb2.ToolStatus.PLAYING_READY_TO_CHECK
        else:
            tool_status.recording_status = runtime_status_pb2.ToolStatus.RECORDING_READY_TO_CHECK
            tool_status.playing_status = runtime_status_pb2.ToolStatus.PLAYING_NOT_READY
        cls._calculate()

    @classmethod
    def get_module(cls, module_name):
        """Get module status by name."""
        if cls.module_dict.get(module_name) is None:
            # Init module status for once.
            module_status = cls.pb_singleton.modules.add(name=module_name)
            cls.module_dict[module_name] = module_status
        return cls.module_dict[module_name]

    @classmethod
    def get_hardware(cls, hardware_name):
        """Get harware status by name."""
        if cls.hardware_dict.get(hardware_name) is None:
            # Init hardware status for once.
            hardware_status = cls.pb_singleton.hardware.add(name=hardware_name)
            cls.hardware_dict[hardware_name] = hardware_status
        return cls.hardware_dict[hardware_name]

    @classmethod
    def get_tools(cls):
        """Get tools status."""
        return cls.pb_singleton.tools

    @classmethod
    def status_json(cls):
        """Convert status to json dict."""

        def pb_to_json(pb, include_default_values=False):
            """Convert proto to json dict."""
            return json_format.MessageToDict(pb, include_default_values, True)

        def pb_dict_to_json(pb_dict):
            """Convert {key: value_pb} to {key, value_dict}."""
            return {
                key: pb_to_json(value_pb)
                for key, value_pb in pb_dict.iteritems()
            }

        return {
            'timestamp': cls._current_timestamp(),
            'modules': pb_dict_to_json(cls.module_dict),
            'hardware': pb_dict_to_json(cls.hardware_dict),
            'tools': pb_to_json(cls.pb_singleton.tools, True),
        }

    @classmethod
    def broadcast_status_if_changed(cls):
        """Broadcast status change."""
        cls._calculate()
        new_fingerprint = hash(str(cls.pb_singleton))
        if cls.pb_fingerprint != new_fingerprint:
            flask_socketio.emit(
                'new_status',
                cls.status_json(),
                broadcast=True,
                namespace='/runtime_status')
            cls.pb_fingerprint = new_fingerprint

    @classmethod
    def _calculate(cls):
        """Update runtime status fields which need to be calculated."""
        conf_pb = config.Config.get_pb()

        modules_and_hardware_ready = cls.are_all_modules_ready(
        ) and cls.are_all_hardware_ready()
        cls._calculate_recording_status(modules_and_hardware_ready)
        cls._calculate_playing_status(modules_and_hardware_ready)
        cls._calculate_guide_message()

    @classmethod
    def are_all_modules_ready(cls):
        """Check if all modules are ready."""
        for mod in config.Config.get_pb().modules:
            mod_status = cls.get_module(mod.name).status
            if mod_status != runtime_status_pb2.ModuleStatus.STARTED:
                return False
        return True

    @classmethod
    def are_all_hardware_ready(cls):
        """Check if all modules are ready."""
        for hw in config.Config.get_pb().hardware:
            hw_status = cls.get_hardware(hw.name).status
            if hw_status != int(runtime_status_pb2.HardwareStatus.OK):
                return False
        return True

    @classmethod
    def stat_playable_duration(cls):
        """Stat playable duration."""
        file_to_play = config.Config.get_realpath(gflags.FLAGS.file_to_play)
        if os.path.exists(file_to_play):
            with open(file_to_play, 'r') as f:
                kFreq = 100
                cls.playable_duration = sum([1 for line in f]) / kFreq
        else:
            cls.playable_duration = 0
        return cls.playable_duration

    @classmethod
    def _calculate_recording_status(cls, modules_and_hardware_ready):
        """Calculate recording status."""
        CHECKING = runtime_status_pb2.ToolStatus.RECORDING_CHECKING
        READY_TO_START = runtime_status_pb2.ToolStatus.RECORDING_READY_TO_START

        tool_status = cls.get_tools()
        recording_status = tool_status.recording_status
        if recording_status == CHECKING and modules_and_hardware_ready:
            tool_status.recording_status = READY_TO_START
        elif recording_status == READY_TO_START and not modules_and_hardware_ready:
            tool_status.recording_status = CHECKING

    @classmethod
    def _calculate_playing_status(cls, modules_and_hardware_ready):
        """Calculate playing status."""
        ToolStatus = runtime_status_pb2.ToolStatus
        tool_status = cls.get_tools()

        playing_status = tool_status.playing_status
        if tool_status.playing_status == ToolStatus.PLAYING_NOT_READY:
            if tool_status.recording_status == ToolStatus.RECORDING_FINISHED:
                if cls.playable_duration > 0:
                    tool_status.playing_status = ToolStatus.PLAYING_READY_TO_CHECK
                else:
                    glog.info(
                        'RuntimeStatus::_calculate_playing_status: No file to play'
                    )
        elif (playing_status == ToolStatus.PLAYING_CHECKING
              and modules_and_hardware_ready
              and tool_status.planning_ready):
            tool_status.playing_status = ToolStatus.PLAYING_READY_TO_START
            glog.info(
                'RuntimeStatus::_calculate_playing_status: All modules/hardware are ready'
            )
        elif playing_status == ToolStatus.PLAYING_READY_TO_START and not (
                modules_and_hardware_ready and tool_status.planning_ready):
            tool_status.playing_status = ToolStatus.PLAYING_CHECKING
            glog.info('RuntimeStatus::_calculate_playing_status: ' \
                      'Not all modules/hardware are ready')

    @classmethod
    def _calculate_guide_message(cls):
        """Update guide message according to status."""
        ToolStatus = runtime_status_pb2.ToolStatus
        tool_status = cls.get_tools()

        if tool_status.recording_status == ToolStatus.RECORDING_READY_TO_CHECK:
            tool_status.message = 'Before recording, you need to setup the system.'
        elif tool_status.recording_status == ToolStatus.RECORDING_CHECKING:
            tool_status.message = 'Waiting for modules and hardware.'
        elif tool_status.recording_status == ToolStatus.RECORDING_READY_TO_START:
            tool_status.message = 'Now you are ready to record.'
        elif (tool_status.recording_status == ToolStatus.RECORDING or
              tool_status.playing_status == ToolStatus.PLAYING):
            if not tool_status.message.isdigit():
                # The timestamp we started recording or playing.
                tool_status.message = str(cls._current_timestamp())
        elif (tool_status.recording_status == ToolStatus.RECORDING_FINISHED and
              tool_status.playing_status == ToolStatus.PLAYING_NOT_READY):
            tool_status.message = 'The recorded data/log/garage.csv is invalid.<br/>' \
                                  'Please fix, or simply try recording again by clicking "New".'
        elif tool_status.playing_status == ToolStatus.PLAYING_CHECKING:
            if tool_status.planning_ready:
                tool_status.message = 'Waiting for modules and hardware.'
            else:
                tool_status.message = 'Waiting for planning ready.'
        elif tool_status.playing_status == ToolStatus.PLAYING_READY_TO_CHECK:
            tool_status.message = '{}s of driving was recorded.<br/>' \
                                  'Before playing, please setup the system.' \
                                  .format(cls.playable_duration)
        elif tool_status.playing_status == ToolStatus.PLAYING_READY_TO_START:
            tool_status.message = 'Make sure the OPERATOR is ready! Now you are good to go.'
        else:
            tool_status.message = 'Something goes wrong.<br/>Please record again or RESET ALL.'

    @classmethod
    def _current_timestamp(cls):
        """Current timestamp in milliseconds."""
        return int(time.time() * 1000)
