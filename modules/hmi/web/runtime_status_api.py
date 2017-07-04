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

"""Runtime status restful Api."""

import httplib

import flask
import flask_restful
import glog
import google.protobuf.json_format as json_format

import modules.hmi.proto.runtime_status_pb2 as runtime_status_pb2
import runtime_status


class RuntimeStatusApi(flask_restful.Resource):
    """HMI runtime status api."""

    def get(self):
        """Get global runtime status."""
        glog.info('RuntimeStatusApi: Get global status.')
        return runtime_status.RuntimeStatus.status_json()

    def post(self):
        """
        Clients report runtime status.

        The client must PUT a json object which can be converted to
        runtime_status_pb2.RuntimeStatus.
        """
        try:
            new_status = json_format.ParseDict(
                flask.request.get_json(), runtime_status_pb2.RuntimeStatus())
        except Exception as e:
            error_msg = 'RuntimeStatusApi: Cannot parse given data "{}"'.format(
                flask.request.get_json())
            glog.error(error_msg)
            return error_msg, httplib.BAD_REQUEST

        # Merge status.
        glog.info('RuntimeStatusApi: Put status:\n{}'.format(new_status))
        cur_status = runtime_status.RuntimeStatus
        for module in new_status.modules:
            cur_status.get_module(module.name).MergeFrom(module)
        for hardware in new_status.hardware:
            cur_status.get_hardware(hardware.name).MergeFrom(hardware)
        cur_status.get_tools().MergeFrom(new_status.tools)

        cur_status.broadcast_status_if_changed()
        return 'OK', httplib.OK
