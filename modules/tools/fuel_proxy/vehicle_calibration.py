#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

""" Vehicle calibration with fuel service. """

import json
import os

from absl import app
from absl import flags
from absl import logging
import google.protobuf.json_format as json_format
import google.protobuf.text_format as text_format
import requests

from modules.tools.fuel_proxy.proto.job_config_pb2 import JobConfig


flags.DEFINE_string('fuel_proxy', None, 'Endpoint of Apollo-Fuel proxy.')
flags.DEFINE_string('job_config', None, 'Vehicle calibration job config.')


class VehicleCalibration(object):
    """ Vehicle calibration with fuel service. """

    def __init__(self):
        self.job_config = None
        self.ssl_cert = os.path.join(os.path.dirname(__file__), 'conf/cert.pem')

    def parse_input(self):
        """Parse and sanity check input data."""
        # Read and parse job_config
        job_config_file = flags.FLAGS.job_config
        if not job_config_file:
            logging.error('Please provide proper --job_config.')
            return False
        if not os.path.exists(job_config_file):
            logging.error('The given job_config file not exist.')
            return False
        try:
            with open(job_config_file, 'r') as fin:
                self.job_config = text_format.Merge(fin.read(), JobConfig())
        except Exception as e:
            logging.error('Failed to parse job_config: {}'.format(e))
            return False

        logging.info('Parsed input successfully!')
        return True

    def send_request(self):
        """Send job config to remote fuel proxy."""
        if not flags.FLAGS.fuel_proxy:
            logging.error('Please provide proper --fuel_proxy.')
            return
        request_json = json_format.MessageToJson(
            self.job_config, preserving_proto_field_name=True)
        request = requests.post(flags.FLAGS.fuel_proxy, json=request_json,
                                verify=self.ssl_cert)
        if request.ok:
            response = json.loads(request.json())
            logging.info('OK: {}'.format(response.get('message')))
        else:
            logging.error('Request failed with HTTP code {}'.format(
                request.status_code))


def main(argv):
    """Main process."""
    calibration = VehicleCalibration()
    if not calibration.parse_input():
        return
    calibration.send_request()

if __name__ == '__main__':
    app.run(main)
