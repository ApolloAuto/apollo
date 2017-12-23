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
Stat acc in time sequence.
Usage:
    ./stat_acc.py bag1 bag2 ...
"""

import sys
import os
import datetime

from rosbag.bag import Bag


kChassisTopic = '/apollo/canbus/chassis'
kLocalizationTopic = '/apollo/localization/pose'
kNano = 10 ** 9


class AccCalculator(object):
    """Calculate mileage."""

    def __init__(self):
        """Init."""
        self.auto_mileage = 0.0
        self.manual_mileage = 0.0
        self.disengagements = 0

    def calculate(self, bag_file, log_file):
        """Calculate and log acc."""
        last_t = None
        last_max_abs_acc = 0
        last_speed = None
        with Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[kChassisTopic]):
                if last_t is not None:
                    acc = (msg.speed_mps - last_speed) / (
                    t.to_sec() - last_t.to_sec())
                    print "acc = ", acc, "delta_t = ", (t.to_sec() - last_t.to_sec())
                    if int(t.to_sec()) != int(last_t.to_sec()):
                        log_file.write(str(int(round(last_t.to_sec())))
                                       + "\t" + str(last_max_abs_acc) + "\n")
                        last_max_abs_acc = acc
                    else:
                        if abs(acc) > abs(last_max_abs_acc):
                            last_max_abs_acc = acc
                last_t = t
                last_speed = msg.speed_mps


def main():
    """Main function."""
    pgm_path = os.path.dirname(os.path.realpath(__file__))
    now = datetime.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
    log_file = open(pgm_path + "/" + now + ".txt", "w")

    mc = AccCalculator()
    for bag_file in sys.argv[1:]:
        mc.calculate(bag_file, log_file)


if __name__ == '__main__':
    main()
