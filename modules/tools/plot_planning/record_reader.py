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

from cyber.python.cyber_py3.record import RecordReader
from modules.common_msgs.chassis_msgs import chassis_pb2
from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.planning_msgs import planning_pb2


class RecordItemReader:
    def __init__(self, record_file):
        self.record_file = record_file

    def read(self, topics):
        reader = RecordReader(self.record_file)
        for msg in reader.read_messages():
            if msg.topic not in topics:
                continue
            if msg.topic == "/apollo/canbus/chassis":
                chassis = chassis_pb2.Chassis()
                chassis.ParseFromString(msg.message)
                data = {"chassis": chassis}
                yield data

            if msg.topic == "/apollo/localization/pose":
                location_est = localization_pb2.LocalizationEstimate()
                location_est.ParseFromString(msg.message)
                data = {"pose": location_est}
                yield data

            if msg.topic == "/apollo/planning":
                planning = planning_pb2.ADCTrajectory()
                planning.ParseFromString(msg.message)
                data = {"planning": planning}
                yield data
