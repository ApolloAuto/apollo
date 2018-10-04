#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

import sys
import numpy as np
from cybertron.record import RecordReader
from modules.control.proto import control_cmd_pb2
from modules.planning.proto import planning_pb2
from modules.canbus.proto import chassis_pb2


class bcolors:
    """ output color schema"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def statistical_analyzer(data):
    """ statistical analyzer"""
    arr = np.array(data)

    v = np.average(arr)
    print bcolors.OKBLUE + "Average: \t" + bcolors.ENDC, "{0:.2f}".format(v)

    std = np.std(arr)
    print bcolors.OKBLUE + "STD: \t\t" + bcolors.ENDC, "{0:.2f}".format(std)

    p = np.percentile(arr, 10)
    print bcolors.OKBLUE + "10 Percentile: \t" + bcolors.ENDC, \
        "{0:.2f}".format(p)

    p = np.percentile(arr, 50)
    print bcolors.OKBLUE + "50 Percentile: \t" + bcolors.ENDC, \
        "{0:.2f}".format(p)

    p = np.percentile(arr, 99)
    print bcolors.OKBLUE + "90 Percentile: \t" + bcolors.ENDC, \
        "{0:.2f}".format(p)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "usage: python main.py record_file"
    record_file = sys.argv[1]
    reader = RecordReader(record_file)

    control_latency_list = []
    planning_latency_list = []
    auto_drive = False
    for msg in reader.read_messages():
        if msg.topic == "/apollo/canbus/chassis":
            chassis = chassis_pb2.Chassis()
            chassis.ParseFromString(msg.message)
            if chassis.driving_mode == \
                    chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
                auto_drive = True
            else:
                auto_drive = False

        if msg.topic == "/apollo/control":
            if auto_drive:
                continue
            control_cmd = control_cmd_pb2.ControlCommand()
            control_cmd.ParseFromString(msg.message)
            control_latency_list.append(
                control_cmd.latency_stats.total_time_ms)

        if msg.topic == "/apollo/planning":
            if auto_drive:
                continue
            adc_trajectory = planning_pb2.ADCTrajectory()
            adc_trajectory.ParseFromString(msg.message)
            latency = adc_trajectory.latency_stats.total_time_ms
            planning_latency_list.append(latency)

    print ""
    print bcolors.HEADER + "--- Control Latency (ms) ---" + bcolors.ENDC
    statistical_analyzer(control_latency_list)

    print ""
    print bcolors.HEADER + "--- Planning Latency (ms) ---" + bcolors.ENDC
    statistical_analyzer(planning_latency_list)
