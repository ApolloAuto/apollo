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
This module provide function to plot the speed control info from log csv file
"""

import math
import warnings

import numpy as np
import scipy.signal as signal


warnings.simplefilter('ignore', np.RankWarning)

SPEED_INTERVAL = 0.2
SPEED_DELAY = 130  # Speed report delay relative to IMU information


def preprocess(filename):
    data = np.genfromtxt(filename, delimiter=',', names=True)
    data = data[np.where(data['io'] == 0)[0]]
    data = data[np.argsort(data['time'])]
    data['time'] = data['time'] - data['time'][get_start_index(data)]

    b, a = signal.butter(6, 0.05, 'low')
    data['imu'] = signal.filtfilt(b, a, data['imu'])

    data['imu'] = np.append(data['imu'][-SPEED_DELAY // 10:],
                            data['imu'][0:-SPEED_DELAY // 10])
    return data


def get_start_index(data):
    if np.all(data['vehicle_speed'] == 0):
        return 0

    start_ind = np.where(data['brake_percentage'] == 40)

    if len(start_ind[0] > 0):
        ind = start_ind[0][0]
        while ind < len(data):
            if data['brake_percentage'][ind] == 40:
                ind += 1
            else:
                break
        return ind
    else:
        ind = 0
        while ind < len(data):
            if abs(data['vehicle_speed'][ind]) < 0.01:
                ind += 1
            else:
                break
        return ind


def process(data):
    """
    process data
    """
    np.set_printoptions(precision=3)

    if np.all(data['vehicle_speed'] == 0):
        print("All Speed = 0")
        return [], [], [], [], [], []

    start_index = get_start_index(data)

    # print "Start index: ", start_index
    data = data[start_index:]
    data['time'] = data['time'] - data['time'][0]

    transition = np.where(
        np.logical_or(
            np.diff(data['ctlbrake']) != 0, np.diff(data['ctlthrottle']) != 0))[
                0]
    transition = np.insert(np.append(transition, len(data) - 1), 0, 0)
    # print "Transition indexes: ", transition

    speedsegments = []
    timesegments = []
    accsegments = []
    tablespeed = []
    tableacc = []
    tablecmd = []

    for i in range(len(transition) - 1):
        # print "process transition index:", data['time'][transition[i]], ":", data['time'][transition[i + 1]]
        speedsection = data['vehicle_speed'][transition[i]:transition[i +
                                                                      1] + 1]
        timesection = data['time'][transition[i]:transition[i + 1] + 1]
        brake = data['ctlbrake'][transition[i] + 1]
        throttle = data['ctlthrottle'][transition[i] + 1]
        imusection = data['imu'][transition[i]:transition[i + 1] + 1]
        if brake == 0 and throttle == 0:
            continue
        # print "Brake CMD: ", brake, " Throttle CMD: ", throttle
        firstindex = 0

        while speedsection[firstindex] == 0:
            firstindex += 1
        firstindex = max(firstindex - 2, 0)
        speedsection = speedsection[firstindex:]
        timesection = timesection[firstindex:]
        imusection = imusection[firstindex:]

        if speedsection[0] < speedsection[-1]:
            is_increase = True
            lastindex = np.argmax(speedsection)
        else:
            is_increase = False
            lastindex = np.argmin(speedsection)

        speedsection = speedsection[0:lastindex + 1]
        timesection = timesection[0:lastindex + 1]
        imusection = imusection[0:lastindex + 1]

        speedmin = np.min(speedsection)
        speedmax = np.max(speedsection)
        speedrange = np.arange(
            max(0, round(speedmin / SPEED_INTERVAL) * SPEED_INTERVAL),
            min(speedmax, 10.01), SPEED_INTERVAL)
        # print "Speed min, max", speedmin, speedmax, is_increase, firstindex, lastindex, speedsection[-1]
        accvalue = []
        for value in speedrange:
            val_ind = 0
            if is_increase:
                while val_ind < len(
                        speedsection) - 1 and value > speedsection[val_ind]:
                    val_ind += 1
            else:
                while val_ind < len(
                        speedsection) - 1 and value < speedsection[val_ind]:
                    val_ind += 1
            if val_ind == 0:
                imu_value = imusection[val_ind]
            else:
                slope = (imusection[val_ind] - imusection[val_ind - 1]) / (
                    speedsection[val_ind] - speedsection[val_ind - 1])
                imu_value = imusection[val_ind - 1] + slope * (
                    value - speedsection[val_ind - 1])
            accvalue.append(imu_value)

        if brake == 0:
            cmd = throttle
        else:
            cmd = -brake
        # print "Overall CMD: ", cmd
        # print "Time: ", timesection
        # print "Speed: ", speedrange
        # print "Acc: ", accvalue
        # print cmd
        tablecmd.append(cmd)
        tablespeed.append(speedrange)
        tableacc.append(accvalue)

        speedsegments.append(speedsection)
        accsegments.append(imusection)
        timesegments.append(timesection)

    return tablecmd, tablespeed, tableacc, speedsegments, accsegments, timesegments
