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

"""Helper to report status to HMI."""

import gflags
import glog
import google.protobuf.json_format as json_format
import requests

# Make sure <apollo>/bazel-genfiles is in your PYTHONPATH.
import modules.hmi.proto.runtime_status_pb2 as runtime_status_pb2

gflags.DEFINE_string('hmi_runtime_status_api',
                     'http://127.0.0.1:8887/runtime_status_api',
                     'Address of HMI runtime status restful api.')


class HMIStatusHelper(object):
    """Helper to report status to HMI."""

    @staticmethod
    def report_hardware_status(hardware_status_list):
        """Report hardware status to HMI."""
        status_pb = runtime_status_pb2.RuntimeStatus()
        for hardware_status in hardware_status_list:
            status_pb.hardware.add().MergeFrom(hardware_status)

        json_dict = json_format.MessageToDict(status_pb, False, True)
        try:
            req = requests.post(
                gflags.FLAGS.hmi_runtime_status_api, json=json_dict)
            glog.info('Put HardwareStatus: {}'.format(req.json()))
        except Exception as e:
            glog.error('Failed to put HardwareStatus: {}'.format(e))

    @staticmethod
    def report_status(status):
        """Report status to HMI."""
        json_dict = json_format.MessageToDict(status, False, True)
        try:
            req = requests.post(
                gflags.FLAGS.hmi_runtime_status_api, json=json_dict)
            glog.info('Put RuntimeStatus: {}'.format(req.json()))
        except Exception as e:
            glog.error('Failed to put RuntimeStatus: {}'.format(e))
