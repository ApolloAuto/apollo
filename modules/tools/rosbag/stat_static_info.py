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
"""
Stat static info.
Usage:
    ./stat_static_info.py <bag_file>
    ./stat_static_info.py <task_dir>  # <task_dir> contains a list of bags.
"""

import os
import sys

from modules.data.proto.static_info_pb2 import StaticInfo
from rosbag.bag import Bag

kStaticInfoTopic = '/apollo/monitor/static_info'
kTopics = [
    kStaticInfoTopic,
]


class StaticInfoCalculator(object):
    """Stat static info."""

    def __init__(self):
        self.vehicle_name = None
        self.vehicle_vin = None

    def process_file(self, bag_file):
        """
        Extract information from bag file.
        Return True if we are done collecting all information.
        """
        try:
            with Bag(bag_file, 'r') as bag:
                for _, msg, _ in bag.read_messages(topics=kTopics):
                    if msg.vehicle.name:
                        self.vehicle_name = msg.vehicle.name.lower()
                    if msg.vehicle.license.vin:
                        self.vehicle_vin = msg.vehicle.license.vin
                    if self.done():
                        return True
        except:
            return False
        return self.done()

    def process_dir(self, bag_dir):
        """Process a directory."""
        files = []
        dirs = []
        for f in os.listdir(bag_dir):
            f_path = os.path.join(bag_dir, f)
            if os.path.isfile(f_path):
                files.append(f_path)
            elif os.path.isdir(f_path):
                dirs.append(f_path)
            # Ignore links.

        # Reverse sort the bags or dirs, trying to get info from the latest.
        for bag in sorted(files, reverse=True):
            if self.process_file(bag):
                return True
        for subdir in sorted(dirs, reverse=True):
            if self.process_dir(subdir):
                return True
        return False

    def done(self):
        """Check if all info are collected."""
        # Currently we only care about vehicle name.
        return bool(self.vehicle_name)


def main(path):
    """Process a path."""
    calc = StaticInfoCalculator()
    if os.path.isfile(path):
        calc.process_file(path)
    else:
        calc.process_dir(path)

    # Output result, which might be None
    print 'vehicle_name:', calc.vehicle_name
    print 'vehicle_vin:', calc.vehicle_vin

if __name__ == '__main__':
    main(sys.argv[1])
