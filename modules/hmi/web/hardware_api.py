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

"""Hardware restful Api."""

import httplib

import flask
import flask_restful
import glog

import config
import modules.hmi.proto.runtime_status_pb2 as runtime_status_pb2
import runtime_status
import system_cmd


class HardwareApi(flask_restful.Resource):
    """Hardware in Apollo."""

    def post(self, hardware_name):
        """Query on hardware."""
        args = flask.request.form
        if args.get('execute_command'):
            return HardwareApi.execute_cmd(hardware_name,
                                           args['execute_command'])

        msg = 'Unknown query: {}'.format(args)
        glog.error(msg)
        return msg, httplib.BAD_REQUEST

    @staticmethod
    def execute_cmd(hardware_name, cmd_name):
        """Execute hardware command."""
        # Run command on all hardware if the name is 'all'.
        if hardware_name == 'all':
            for hw_conf in config.Config.get_pb().hardware:
                HardwareApi._run_command(hw_conf, cmd_name)
            runtime_status.RuntimeStatus.broadcast_status_if_changed()
            return 'OK', httplib.OK

        # Or else, run command on the specified hardware.
        conf = config.Config.get_hardware(hardware_name)
        if conf is None:
            msg = 'Cannot find config for hardware {}'.format(hardware_name)
            glog.error(msg)
            return msg, httplib.BAD_REQUEST
        result = HardwareApi._run_command(conf, cmd_name)
        runtime_status.RuntimeStatus.broadcast_status_if_changed()
        return result

    @staticmethod
    def _run_command(conf, cmd_name):
        """Implementation of running command on hardware."""
        cmd = next((cmd for cmd in conf.supported_commands
                    if cmd.name == cmd_name), None)
        if cmd is None:
            msg = 'Cannot find command {} for hardware {}'.format(
                cmd_name, conf.name)
            glog.error(msg)
            return msg, httplib.BAD_REQUEST

        # Construct the command string by joining all components.
        cmd.command[0] = config.Config.get_realpath(cmd.command[0])
        cmd_str = ' '.join(cmd.command)
        system_cmd.run_in_background(cmd_str, cmd.stdout_file, cmd.stderr_file)

        # Update hardware status.
        hardware_status = runtime_status.RuntimeStatus.get_hardware(conf.name)
        if cmd_name == 'health_check':
            hardware_status.status = runtime_status_pb2.HardwareStatus.CHECKING

        return 'OK', httplib.OK
