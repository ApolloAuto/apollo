# -*- coding: utf-8 -*-
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
"""
Usage:
    python body_sensation_evalution.py bag1 bag2 ...
"""

import collections
import math
import sys
import time

from rosbag.bag import Bag

from modules.canbus.proto.chassis_pb2 import Chassis

kChassisTopic = '/apollo/canbus/chassis'
kLocalizationTopic = '/apollo/localization/pose'

class BodySensationCalculator(object):

    def __init__(self):
        self.driving_mode = []
        self._timestamp = 0.0
        self._bump_lock_time = 0.0
        self._last_bump_time = 0.0
        self._last_speed_down_2_time = 0.0
        self._last_speed_down_4_time = 0.0
        self._last_speed_up_2_time = 0.0
        self._last_speed_up_4_time = 0.0
        self._last_turning_time = 0.0
        self._speed_down_2_flag = 0
        self._speed_down_4_flag = 0
        self._speed_up_2_flag = 0
        self._speed_up_4_flag = 0
        self._turning_flag = 0
        self.auto_counts = {}
        self.auto_counts["speed_down_2"] = 0
        self.auto_counts["speed_down_4"] = 0
        self.auto_counts["speed_up_2"] = 0
        self.auto_counts["speed_up_4"] = 0
        self.auto_counts["turning"] = 0
        self.auto_counts["bumps"] = 0
        self.manual_counts = {}
        self.manual_counts["speed_down_2"] = 0
        self.manual_counts["speed_down_4"] = 0
        self.manual_counts["speed_up_2"] = 0
        self.manual_counts["speed_up_4"] = 0
        self.manual_counts["turning"] = 0
        self.manual_counts["bumps"] = 0

    def get_driving_mode(self, bag_file):
        """get driving mode, which is stored in a dict"""
        with Bag(bag_file, 'r') as bag:
            mode={}
            mode["status"] = 'UNKNOW'
            mode["start_time"] = 0.0
            mode["end_time"] = 0.0;  
            for topic, msg, _t in bag.read_messages([kChassisTopic]):
                t = long(str(_t))*pow(10,-9)
                cur_status = msg.driving_mode
                if mode["status"] != cur_status:
                    if mode["status"] != 'UNKNOW':
                        self.driving_mode.append(mode)
                    mode["status"] = cur_status
                    mode["start_time"] = t
                mode["end_time"] = t
            self.driving_mode.append(mode)

    def _check_status(self, timestamp):
        """check driving mode according to timestamp"""
        for mode in self.driving_mode:
            if timestamp >= mode["start_time"] and timestamp <= mode["end_time"]:
                if mode["status"] == Chassis.COMPLETE_AUTO_DRIVE:
                    return True
                else:
                    return False
        return False

    def _bumps_rollback(self, bump_time):
        """rollback 1 second when passing bumps"""

        if bump_time - self._last_speed_down_2_time <= 1:
            if self._check_status(self._last_speed_down_2_time):
                self.auto_counts["speed_down_2"] -= 1
            else:
                self.manual_counts["speed_down_2"] -= 1

        if bump_time - self._last_speed_up_2_time <= 1:
            if self._check_status(self._last_speed_up_2_time):
                self.auto_counts["speed_up_2"] -= 1
            else:
                self.manual_counts["speed_up_2"] -= 1

        if bump_time - self._last_speed_down_4_time <= 1:
            if self._check_status(self._last_speed_down_4_time):
                self.auto_counts["speed_down_4"] -= 1
            else:
                self.manual_counts["speed_down_4"] -= 1

        if bump_time - self._last_speed_up_4_time <= 1:
            if self._check_status(self._last_speed_up_4_time):
                self.auto_counts["speed_up_4"] -= 1
            else:
                self.manual_counts["speed_up_4"] -= 1

        if bump_time - self._last_turning_time <= 1:
            if self._check_status(self._last_turning_time):
                self.auto_counts["turning"] -= 1
            else:
                self.manual_counts["turning"] -= 1


    def calculate(self, bag_file):
        with Bag(bag_file, 'r') as bag:
            for topic, msg, _t in bag.read_messages([kLocalizationTopic]):
                t = long(str(_t))*pow(10,-9)
                #时间戳
                #dt = time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(_t))
                self.timestamp = t
                if t - self._bump_lock_time <= 3:
                    continue
                acc_x = msg.pose.linear_acceleration.x
                acc_y = msg.pose.linear_acceleration.y
                acc_z = msg.pose.linear_acceleration.z
                diff_bump_time = t - self._last_bump_time
                if abs(acc_z) >= 2 and diff_bump_time >= 3:
                    self._bumps_rollback(t)
                    self._last_bump_time = t
                    self._bump_lock_time = t

                    if self._check_status(t):
                        self.auto_counts["bumps"] += 1
                    else:
                        self.manual_counts["bumps"] += 1
                else:
                    if self._speed_down_2_flag:
                        if acc_y <= -4:
                            self._speed_down_4_flag = 1
                            continue
                        if acc_y <= -2:
                            continue
                        if self._speed_down_4_flag == 1 and t - self._last_speed_down_4_time >=1:
                            self._last_speed_down_4_time = t
                            if self._check_status(t):
                                self.auto_counts["speed_down_4"] += 1
                            else:
                                self.manual_counts["speed_down_4"] += 1
                        elif t - self._last_speed_down_2_time >= 1:
                            self._last_speed_down_2_time = t
                            if self._check_status(t):
                                self.auto_counts["speed_down_2"] += 1
                            else:
                                self.manual_counts["speed_down_2"] += 1
                        self._speed_down_2_flag = 0
                        self._speed_down_4_flag = 0
                    elif acc_y <= -2:
                        self._speed_down_2_flag = 1

                    if self._speed_up_2_flag:
                        if acc_y >= 4:
                            self._speed_up_4_flag = 1
                            continue
                        if acc_y >= 2:
                            continue
                        if self._speed_up_4_flag == 1 and t - self._last_speed_up_4_time >=1:
                            self._last_speed_up_4_time = t
                            if self._check_status(t):
                                self.auto_counts["speed_up_4"] += 1
                            else:
                                self.manual_counts["speed_up_4"] += 1
                        elif t - self._last_speed_up_2_time >= 1:
                            self._last_speed_up_2_time = t
                            if self._check_status(t):
                                self.auto_counts["speed_up_2"] += 1
                            else:
                                self.manual_counts["speed_up_2"] += 1
                        self._speed_up_2_flag = 0
                        self._speed_up_4_flag = 0
                    elif acc_y >= -2:
                        self._speed_up_2_flag = 1

                    if self._turning_flag:
                        if abs(acc_x) >= 2:
                            continue
                        if t - self._last_turning_time >= 1:
                            self._last_turning_time = t
                            if self._check_status(t):
                                self.auto_counts["turning"] += 1
                            else:
                                self.manual_counts["speed_up_2"] += 1
                        self._turning_flag = 0
                    elif abs(acc_x) >= -2:
                        self._turning_flag = 1


def main():
    """Main function."""
    bsc = BodySensationCalculator()
    for bag_file in sys.argv[1:]:
        bsc.get_driving_mode(bag_file)
    for bag_file in sys.argv[1:]:
        bsc.calculate(bag_file)
    counts = {}
    counts["auto"] = sorted(bsc.auto_counts.items())
    counts["manual"] = sorted(bsc.manual_counts.items())
    print(counts)

if __name__ == '__main__':
    main()
