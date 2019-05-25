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

import os

from absl import app
from absl import flags
from absl import logging
import google.protobuf.text_format as text_format

from modules.tools.fuel_proxy.proto.job_config_pb2 import JobConfig


flags.DEFINE_string('fuel_proxy', None, 'Endpoint of Apollo-Fuel proxy.')
flags.DEFINE_string('job_config', None, 'Vehicle calibration job config.')


class VehicleCalibration(object):
    """ Vehicle calibration with fuel service. """

    def __init__(self):
        self.job_config = None
        if not self.sanity_check():
            return None

    def sanity_check(self):
        """Sanity check input data."""
        # Read and parse job_config
        job_config = flags.FLAGS.job_config
        if not job_config:
            logging.fatal('Please provide proper --job_config.')
            return False
        if not os.path.exists(job_config):
            logging.fatal('The given job_config file not exist.')
            return False
        try:
            with open(job_config, 'r') as fin:
                self.job_config = text_format.Merge(fin.read(), JobConfig())
        except Exception as e:
            logging.fatal('Failed to parse job_config: {}'.format(e))
            return False

        return True

def main(argv):
    calibration = VehicleCalibration()
    if calibration is None:
        return


if __name__ == '__main__':
    app.run(main)
