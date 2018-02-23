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

import sys

import numpy as np

import common.proto_utils as proto_utils
from modules.control.proto import calibration_table_pb2
from modules.control.proto.control_conf_pb2 import ControlConf


def load_calibration_raw_data(fn):
    speed_table = {}
    f = open(fn, 'r')
    for line in f:
        items = line.split(',')
        cmd = round(float(items[0]))
        speed = float(items[1])
        acc = round(float(items[2]), 2)
        if speed in speed_table:
            cmd_table = speed_table[speed]
            if cmd in cmd_table:
                cmd_table[cmd].append(acc)
            else:
                cmd_table[cmd] = [acc]
        else:
            cmd_table = {}
            cmd_table[cmd] = [acc]
            speed_table[speed] = cmd_table
    f.close()
    for speed, cmd_table in speed_table.items():
        new_cmd_table = {}
        for cmd, acc_list in cmd_table.items():
            new_cmd_table[cmd] = round(np.mean(acc_list), 2)
        speed_table[speed] = new_cmd_table

    speed_table2 = {}
    for speed, cmd_table in speed_table.items():
        acc_table = {}
        for cmd, acc in cmd_table.items():
            if acc in acc_table:
                acc_table[acc].append(cmd)
            else:
                acc_table[acc] = [cmd]
        speed_table2[speed] = acc_table
    return speed_table2


def load_calibration_raw_data_old(fn):
    speed_table = {}
    f = open(fn, 'r')
    for line in f:
        items = line.split(',')
        cmd = round(float(items[0]))
        speed = float(items[1])
        acc = round(float(items[2]), 2)
        if speed in speed_table:
            acc_table = speed_table[speed]
            if acc in acc_table:
                acc_table[acc].append(cmd)
            else:
                acc_table[acc] = [cmd]
        else:
            acc_table = {}
            acc_table[acc] = [cmd]
            speed_table[speed] = acc_table
    f.close()
    return speed_table


def get_calibration_table_pb(speed_table):
    calibration_table_pb = calibration_table_pb2.ControlCalibrationTable()
    speeds = speed_table.keys()
    speeds.sort()
    for speed in speeds:
        acc_table = speed_table[speed]
        accs = acc_table.keys()
        accs.sort()
        for acc in accs:
            cmds = acc_table[acc]
            cmd = np.mean(cmds)
            item = calibration_table_pb.calibration.add()
            item.speed = speed
            item.acceleration = acc
            item.command = cmd
    return calibration_table_pb

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python plot_results.py old_control_conf.pb.txt result.csv")
        sys.exit(0)

    ctl_conf_pb = proto_utils.get_pb_from_text_file(sys.argv[1], ControlConf())
    speed_table_dict = load_calibration_raw_data(sys.argv[2])
    calibration_table_pb = get_calibration_table_pb(speed_table_dict)
    ctl_conf_pb.lon_controller_conf.calibration_table.CopyFrom(
        calibration_table_pb)

    with open('control_conf.pb.txt', 'w') as f:
        f.write(str(ctl_conf_pb))
