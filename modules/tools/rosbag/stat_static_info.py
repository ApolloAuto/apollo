#!/usr/bin/env python3

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
"""
Stat static info.
Usage:
    ./stat_static_info.py <record_file>
    ./stat_static_info.py <task_dir>  # <task_dir> contains a list of records.
"""

import os
import sys

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.dreamview_msgs import hmi_status_pb2


kChassisInfoTopic = '/apollo/canbus/chassis'
kHMIInfoTopic = '/apollo/hmi/status'


class StaticInfoCalculator(object):
    """
    Stat static information
    """

    def __init__(self):
        self.vehicle_name = None
        self.vehicle_vin = None

    def process_file(self, record_file):
        """
        Extract information from record file.
        Return True if we are done collecting all information.
        """
        try:
            reader = RecordReader(record_file)
            print("Begin to process record file {}".format(record_file))
            for msg in reader.read_messages():
                print(msg.topic)
                if msg.topic == kChassisInfoTopic and self.vehicle_vin is None:
                    chassis = chassis_pb2.Chassis()
                    chassis.ParseFromString(msg.message)
                    if chassis.license.vin:
                        self.vehicle_vin = chassis.license.vin
                elif msg.topic == kHMIInfoTopic and self.vehicle_name is None:
                    hmistatus = hmi_status_pb2.HMIStatus()
                    hmistatus.ParseFromString(msg.message)
                    if hmistatus.current_map:
                        self.vehicle_name = hmistatus.current_map
                        print(self.vehicle_name)
                if self.done():
                    return True
        except:
            return False
        print("Finished processing record file {}".format(record_file))
        return self.done()

    def process_dir(self, record_dir):
        """
        Process a directory
        """
        files = []
        dirs = []
        for f in os.listdir(record_dir):
            f_path = os.path.join(record_dir, f)
            if os.path.isfile(f_path):
                files.append(f_path)
            elif os.path.isdir(f_path):
                dirs.append(f_path)
            # Ignore links.

        # Reverse sort the records or dirs, trying to get info from the latest.
        for record in sorted(files, reverse=True):
            if self.process_file(record):
                return True
        for subdir in sorted(dirs, reverse=True):
            if self.process_dir(subdir):
                return True
        return False

    def done(self):
        """
        Check if all info are collected
        """
        # Currently we only care about vehicle name.
        return bool(self.vehicle_name)


def main():
    """
    Process a path
    """
    if len(sys.argv) < 2:
        print("Usage: %s <record_file|task_dir>" % sys.argv[0])
        sys.exit(0)

    path = sys.argv[1]
    calc = StaticInfoCalculator()
    if os.path.isfile(path):
        calc.process_file(path)
    else:
        calc.process_dir(path)

    # Output result, which might be None
    print('vehicle_name: %s' % calc.vehicle_name)
    print('vehicle_vin: %s' % calc.vehicle_vin)


if __name__ == '__main__':
    main()
