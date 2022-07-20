#!/usr/bin/env python3

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


import sys

import matplotlib.pyplot as plt

from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.chassis_msgs import chassis_pb2


def process(reader):
    last_steering_percentage = None
    last_speed_mps = None
    last_timestamp_sec = None
    speed_data = []
    d_steering_data = []

    for msg in reader.read_messages():
        if msg.topic == "/apollo/canbus/chassis":
            chassis = chassis_pb2.Chassis()
            chassis.ParseFromString(msg.message)

            steering_percentage = chassis.steering_percentage
            speed_mps = chassis.speed_mps
            timestamp_sec = chassis.header.timestamp_sec

            if chassis.driving_mode != chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
                last_steering_percentage = steering_percentage
                last_speed_mps = speed_mps
                last_timestamp_sec = timestamp_sec
                continue

            if last_timestamp_sec is None:
                last_steering_percentage = steering_percentage
                last_speed_mps = speed_mps
                last_timestamp_sec = timestamp_sec
                continue

            if (timestamp_sec - last_timestamp_sec) > 0.02:
                d_steering = (steering_percentage - last_steering_percentage) \
                    / (timestamp_sec - last_timestamp_sec)
                speed_data.append(speed_mps)
                d_steering_data.append(d_steering)

                last_steering_percentage = steering_percentage
                last_speed_mps = speed_mps
                last_timestamp_sec = timestamp_sec

    return speed_data, d_steering_data


if __name__ == "__main__":
    fns = sys.argv[1:]
    fig, ax = plt.subplots()

    for fn in fns:
        reader = RecordReader(fn)
        speed_data, d_steering_data = process(reader)
        ax.scatter(speed_data, d_steering_data)
    ax.set_xlim(-5, 40)
    ax.set_ylim(-300, 300)
    plt.show()
