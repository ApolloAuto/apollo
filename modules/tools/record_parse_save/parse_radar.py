#!/usr/bin/env python3

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
function to parse radar data from *.record files, created using Apollo-Auto

parsed data is saved to *.txt file, for each scan

currently implementation for:
* Continental ARS-408 radar

"""

import json
import os
import sys

from cyber_py3 import cyber
from cyber_py3 import record
from modules.drivers.proto.conti_radar_pb2 import ContiRadar


class RadarMessageConti408(object):
    def __init__(self):
        self.radarDetectionList = []
        self.timestamp_sec = None
        self.num_of_detections = None
        self.radar_module = None
        self.sequence_num = None
        self.radar_channel = None
        self.additional_notes = None

    def toJSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)


class ContiRadarARS408Detection(object):
    def __init__(self):
        self.clusterortrack = None
        self.obstacle_id = None
        self.longitude_dist = None
        self.lateral_dist = None
        self.longitude_vel = None
        self.lateral_vel = None
        self.rcs = None
        self.dynprop = None
        self.longitude_dist_rms = None
        self.lateral_dist_rms = None
        self.longitude_vel_rms = None
        self.lateral_vel_rms = None
        self.probexist = None
        self.meas_state = None
        self.longitude_accel = None
        self.lateral_accel = None
        self.oritation_angle = None
        self.longitude_accel_rms = None
        self.lateral_accel_rms = None
        self.oritation_angle_rms = None
        self.length = None
        self.width = None
        self.obstacle_class = None


def pull_conti_radar_detections(obs):
    """
    file to convert structure from c++ to python format
    """

    dets = ContiRadarARS408Detection()

    dets.clusterortrack = obs.clusterortrack
    dets.obstacle_id = obs.obstacle_id
    dets.longitude_dist = obs.longitude_dist
    dets.lateral_dist = obs.lateral_dist
    dets.longitude_vel = obs.longitude_vel
    dets.lateral_vel = obs.lateral_vel
    dets.rcs = obs.rcs
    dets.dynprop = obs.dynprop
    dets.longitude_dist_rms = obs.longitude_dist_rms
    dets.lateral_dist_rms = obs.lateral_dist_rms
    dets.longitude_vel_rms = obs.longitude_vel_rms
    dets.lateral_vel_rms = obs.lateral_vel_rms
    dets.probexist = obs.probexist
    dets.meas_state = obs.meas_state
    dets.longitude_accel = obs.longitude_accel
    dets.lateral_accel = obs.lateral_accel
    dets.oritation_angle = obs.oritation_angle
    dets.longitude_accel_rms = obs.longitude_accel_rms
    dets.lateral_accel_rms = obs.lateral_accel_rms
    dets.oritation_angle_rms = obs.oritation_angle_rms
    dets.length = obs.length
    dets.width = obs.width
    dets.obstacle_class = obs.obstacle_class

    return dets

def parse_data(channelname, msg, out_folder):
    """
    parser for record-file data from continental ars-408 radar
    """

    msg_contiradar = ContiRadar()
    msg_contiradar.ParseFromString(msg)
    n = len(msg_contiradar.contiobs)

    detections = []
    radar_msg = RadarMessageConti408()

    head_msg = msg_contiradar.contiobs[0].header
    radar_msg.timestamp_sec = head_msg.timestamp_sec
    radar_msg.num_of_detections = n
    radar_msg.radar_module = head_msg.module_name
    radar_msg.sequence_num = head_msg.sequence_num
    radar_msg.radar_channel = channelname

    for i in range(len(msg_contiradar.contiobs)):
        detections.append(pull_conti_radar_detections(msg_contiradar.contiobs[i]))

    radar_msg.radarDetectionList = detections

    json_data = radar_msg.toJSON()
    tstamp = json_data.split()[-2].ljust(20, '0')

    # write this scan to file
    scan_filename = "radar_scan_" + tstamp.replace('.', '_') + ".txt"
    with open(out_folder + scan_filename, 'w') as outfile:
        outfile.write(json_data)

    return tstamp

