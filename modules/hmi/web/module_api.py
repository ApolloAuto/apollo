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
"""Module API."""

from config import Config
from modules.hmi.proto.runtime_status_pb2 import ModuleStatus
from runtime_status import RuntimeStatus
import system_cmd


class ModuleApi(object):
    """
    Module SocketIO API:
        start [all | record_replay | <module>]
        stop [all | <module>]
    """

    @staticmethod
    def start(*args):
        """SocketIO Api: start(module_name)"""
        if len(args) != 1:
            Config.log.critical('ModuleApi::start bad args')
            return
        module_name = args[0]

        def __single_start(module_name):
            conf = Config.get_module(module_name)
            if conf is None:
                Config.log.critical('ModuleApi::start conf is None')
                return
            cmd = next((cmd for cmd in conf.supported_commands
                        if cmd.name == 'start'), None)
            if cmd is None:
                Config.log.critical('ModuleApi::start cmd is None')
                return
            # Update status and run command.
            RuntimeStatus.get_module(module_name).status = ModuleStatus.STARTED
            system_cmd.async_run_command_pb(cmd)

        if module_name == 'all':
            for conf in Config.get_pb().modules:
                __single_start(conf.name)
        elif module_name == 'record_replay_required_modules':
            for mod in Config.record_replay_required_modules:
                __single_start(mod)
        else:
            __single_start(module_name)
        RuntimeStatus.broadcast_status_if_changed()

    @staticmethod
    def stop(*args):
        """SocketIO Api: stop(module_name)"""
        if len(args) != 1:
            Config.log.critical('ModuleApi::stop bad args')
            return
        module_name = args[0]

        def __single_stop(module_name):
            conf = Config.get_module(module_name)
            if conf is None:
                Config.log.critical('ModuleApi::stop conf is None')
                return
            cmd = next((cmd for cmd in conf.supported_commands
                        if cmd.name == 'stop'), None)
            if cmd is None:
                Config.log.critical('ModuleApi::stop cmd is None')
                return
            # Update status and run command.
            RuntimeStatus.get_module(module_name).status = ModuleStatus.STOPPED
            system_cmd.async_run_command_pb(cmd)

        if module_name == 'all':
            _ = [__single_stop(conf.name) for conf in Config.get_pb().modules]
        else:
            __single_stop(module_name)
        RuntimeStatus.broadcast_status_if_changed()
