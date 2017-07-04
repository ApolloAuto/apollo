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

"""Module restful Api."""

import httplib

import flask
import flask_restful
import glog

import config
import modules.hmi.proto.runtime_status_pb2 as runtime_status_pb2
import runtime_status
import system_cmd


class ModuleApi(flask_restful.Resource):
    """Module in Apollo."""

    def post(self, module_name):
        """Run module command and return HTTP response as (content, status)."""
        args = flask.request.form
        if args.get('execute_command'):
            return ModuleApi.execute_cmd(module_name, args['execute_command'])

        msg = 'Unknown query: {}'.format(args)
        glog.error(msg)
        return msg, httplib.BAD_REQUEST

    @staticmethod
    def execute_cmd(module_name, cmd_name):
        """"""
        # Run command on all modules if the module name is exactly 'all'.
        if module_name == 'all':
            for conf in config.Config.get_pb().modules:
                ModuleApi._run_command(conf, cmd_name)
            runtime_status.RuntimeStatus.broadcast_status_if_changed()
            return 'OK', httplib.OK

        # Or else, run command on the specified module.
        conf = config.Config.get_module(module_name)
        if conf is None:
            msg = 'Cannot find config for module {}'.format(module_name)
            glog.fatal(msg)
            return msg, httplib.BAD_REQUEST
        result = ModuleApi._run_command(conf, cmd_name)
        runtime_status.RuntimeStatus.broadcast_status_if_changed()
        return result

    @staticmethod
    def _run_command(conf, cmd_name):
        """Implementation of running command on module."""
        cmd = next((cmd for cmd in conf.supported_commands
                    if cmd.name == cmd_name), None)
        if cmd is None:
            msg = 'Cannot find command {} for module {}'.format(
                cmd_name, conf.name)
            glog.fatal(msg)
            return msg, httplib.BAD_REQUEST

        # Construct the command string by joining all components.
        cmd.command[0] = config.Config.get_realpath(cmd.command[0])
        cmd_str = ' '.join(cmd.command)
        system_cmd.run_in_background(cmd_str, cmd.stdout_file, cmd.stderr_file)

        # Update module status.
        module_status = runtime_status.RuntimeStatus.get_module(conf.name)
        if cmd_name == 'start':
            module_status.status = runtime_status_pb2.ModuleStatus.STARTED
        elif cmd_name == 'stop':
            module_status.status = runtime_status_pb2.ModuleStatus.STOPPED

        return 'OK', httplib.OK
