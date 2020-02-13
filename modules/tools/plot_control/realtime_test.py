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
"""Real Time ACC Calculate Test Tool Based on Speed"""

import argparse
import datetime
import os
import sys

from cyber_py3 import cyber
from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2


SmoothParam = 9


class RealTimeTest(object):

    def __init__(self):
        self.last_t = None
        self.last_speed = None
        self.last_acc = None
        self.acc = None
        self.accs = []
        self.buff = []
        self.count = 0
        self.acclimit = 0

    def chassis_callback(self, chassis_pb2):
        """Calculate ACC from Chassis Speed"""
        speedmps = chassis_pb2.speed_mps
        t = chassis_pb2.header.timestamp_sec
        #speedkmh = speedmps * 3.6

        self.buff.append(speedmps)
        if len(self.buff) < SmoothParam:
            return
        if self.last_t is not None:
            self.count += 1
            deltt = t - self.last_t
            deltv = sum(self.buff) / len(self.buff) - self.last_speed
            if deltt <= 1e-10:
                deltt = 0.000000000001
                print("delt=0 ", t, ",", self.last_t)
            self.acc = deltv / deltt

            self.accs.append(self.acc)
            if abs(self.acc) > self.acclimit:
                print(t, "\t", (sum(self.buff) / len(self.buff)) * 3.6, "\t",
                      self.acc, "\t", self.count, "\t", self.acclimit)
        self.last_acc = self.acc
        self.last_t = t
        self.last_speed = sum(self.buff) / len(self.buff)
        self.buff = []


if __name__ == '__main__':
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Test car realtime status.",
        prog="RealTimeTest.py",
        usage="python realtime_test.py")
    parser.add_argument(
        "--acc",
        type=int,
        required=False,
        default=2,
        help="Acc limit default must > 0")
    args = parser.parse_args()
    if args.acc < 0:
        print("acc must larger than 0")
    cyber.init()
    rttest = RealTimeTest()
    rttest.acclimit = args.acc
    cyber_node = cyber.Node("RealTimeTest")
    cyber_node.create_reader("/apollo/canbus/chassis",
                             chassis_pb2.Chassis, rttest.chassis_callback)
    cyber_node.spin()
    cyber.shutdown()
