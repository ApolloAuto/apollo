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
Stat disengagements and auto/manual driving mileage.
Usage:
    ./stat_mileage.py bag1 bag2 ...
"""

import collections
import math
import sys

from cyber_py import cyber
from cyber_py.record import RecordReader
from modules.canbus.proto import chassis_pb2
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.localization.proto import localization_pb2


kChassisTopic = '/apollo/canbus/chassis'
kLocalizationTopic = '/apollo/localization/pose'


class MileageCalculator(object):
    """Calculate mileage."""

    def __init__(self):
        """Init."""
        self.auto_mileage = 0.0
        self.manual_mileage = 0.0
        self.disengagements = 0

    def calculate(self, bag_file):
        """Calculate mileage."""
        last_pos = None
        last_mode = 'Unknown'
        mileage = collections.defaultdict(lambda: 0.0)
        chassis = chassis_pb2.Chassis()
        localization = localization_pb2.LocalizationEstimate()
        reader = RecordReader(bag_file)
        for msg in reader.read_messages():
            if msg.topic == kChassisTopic:
                chassis.ParseFromString(msg.message)
                # Mode changed
                if last_mode != chassis.driving_mode:
                    if (last_mode == Chassis.COMPLETE_AUTO_DRIVE and
                            chassis.driving_mode == Chassis.EMERGENCY_MODE):
                        self.disengagements += 1
                    last_mode = chassis.driving_mode
                    # Reset start position.
                    last_pos = None
            elif msg.topic == kLocalizationTopic:
                localization.ParseFromString(msg.message)
                cur_pos = localization.pose.position
                if last_pos:
                    # Accumulate mileage, from xyz-distance to miles.
                    mileage[last_mode] += 0.000621371 * math.sqrt(
                        (cur_pos.x - last_pos.x) ** 2 +
                        (cur_pos.y - last_pos.y) ** 2 +
                        (cur_pos.z - last_pos.z) ** 2)
                last_pos = cur_pos
        self.auto_mileage += mileage[Chassis.COMPLETE_AUTO_DRIVE]
        self.manual_mileage += (mileage[Chassis.COMPLETE_MANUAL] +
                                mileage[Chassis.EMERGENCY_MODE])


def main():
    """Main function."""
    mc = MileageCalculator()
    for bag_file in sys.argv[1:]:
        mc.calculate(bag_file)
    print 'Disengagements: %d' % mc.disengagements
    print 'Auto mileage:   %.3f km / %.3f miles' % (
        mc.auto_mileage * 1.60934, mc.auto_mileage)
    print 'Manual mileage: %.3f km / %.3f miles' % (
        mc.manual_mileage * 1.60934, mc.manual_mileage)


if __name__ == '__main__':
    main()
