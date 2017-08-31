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
"""Hardware API."""

from config import Config
from modules.hmi.proto.runtime_status_pb2 import HardwareStatus
from runtime_status import RuntimeStatus
import system_cmd


class HardwareApi(object):
    """
    Hardware SocketIO API:
        health_check [all | <hardware>]
    """

    @staticmethod
    def health_check(*args):
        """SocketIO Api: health_check(hardware_name)"""
        if len(args) != 1:
            Config.log.critical('HardwareApi::health_check bad args')
            return
        hardware_name = args[0]

        def __single_check(hardware_name):
            conf = Config.get_hardware(hardware_name)
            if conf is None:
                Config.log.critical('HardwareApi::health_check conf is None')
                return
            cmd = next((cmd for cmd in conf.supported_commands
                        if cmd.name == 'health_check'), None)
            if cmd is None:
                Config.log.critical('HardwareApi::health_check cmd is None')
                return

            # Update status and run command.
            RuntimeStatus.get_hardware(hardware_name).status = \
                HardwareStatus.CHECKING
            system_cmd.async_run_command_pb(cmd)

        if hardware_name == 'all':
            _ = [__single_check(conf.name) for conf in Config.get_pb().hardware]
        else:
            __single_check(hardware_name)
        RuntimeStatus.broadcast_status_if_changed()
