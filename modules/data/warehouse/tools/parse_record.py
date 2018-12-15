#!/usr/bin/env python
# -*- coding: UTF-8-*-
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
Parse Cyber record into apollo.data.Record.

Use as command tool: parse_record.py <record>
Use as util lib:     RecordParser.Parse(<record>)
"""

import math
import os
import sys

import gflags
import glog
import utm

from cyber.proto.record_pb2 import Header
from cyber_py.record import RecordReader
from modules.canbus.proto.chassis_pb2 import Chassis
from modules.data.proto.record_pb2 import Record
from modules.localization.proto.localization_pb2 import LocalizationEstimate

gflags.DEFINE_float('pos_sample_min_duration', 2, 'In seconds.')
gflags.DEFINE_float('pos_sample_min_distance', 3, 'In meters.')
gflags.DEFINE_integer('utm_zone_id', 10, 'UTM zone id.')
gflags.DEFINE_string('utm_zone_letter', 'S', 'UTM zone letter.')

kChassisChannel = '/apollo/canbus/chassis'
kDriveEventChannel = '/apollo/drive_event'
kHMIStatusChannel = '/apollo/hmi/status'
kLocalizationChannel = '/apollo/localization/pose'


def utm_distance_meters(pos0, pos1):
    """Return distance of pos0 and pos1 in meters."""
    return math.sqrt((pos0.x - pos1.x) ** 2 +
                     (pos0.y - pos1.y) ** 2 +
                     (pos0.z - pos1.z) ** 2)


class RecordParser(object):
    """Wrapper of a Cyber record."""

    @staticmethod
    def Parse(record_file):
        """Simple interface to parse a cyber record."""
        parser = RecordParser(record_file)
        if not parser.ParseMeta():
            return None
        parser.ParseMessages()
        return parser.record

    def __init__(self, record_file):
        """Init input reader and output record."""
        record_file = os.path.abspath(record_file)
        self.record = Record(path=record_file, dir=os.path.dirname(record_file))

        self._reader = RecordReader(record_file)
        # State during processing messages.
        self._current_driving_mode = None
        self._last_position = None
        # To sample driving path.
        self._last_position_sampled = None
        self._last_position_sampled_time = None

    def ParseMeta(self):
        """
        Parse meta info which doesn't need to scan the record.
        Currently we parse the record ID, header and channel list here.
        """
        self.record.header.ParseFromString(self._reader.get_headerstring())
        for chan in self._reader.get_channellist():
            self.record.channels[chan] = self._reader.get_messagenumber(chan)
        if len(self.record.channels) == 0:
            glog.error('No message found in record')
            return False
        return True

    def ParseMessages(self):
        """Process all messages."""
        for channel, msg, _type, timestamp in self._reader.read_messages():
            if channel == kHMIStatusChannel:
                self.ProcessHMIStatus(msg)
            elif channel == kDriveEventChannel:
                self.ProcessDriveEvent(msg)
            elif channel == kChassisChannel:
                self.ProcessChassis(msg)
            elif channel == kLocalizationChannel:
                self.ProcessLocalization(msg)

    def ProcessHMIStatus(self, msg):
        """Save HMIStatus."""
        # Keep the first message and assume it doesn't change in one recording.
        if not self.record.HasField('hmi_status'):
            self.record.hmi_status.ParseFromString(msg)

    def ProcessDriveEvent(self, msg):
        """Save DriveEvents."""
        self.record.drive_events.add().ParseFromString(msg)

    def ProcessChassis(self, msg):
        """Process Chassis, save disengagements."""
        chassis = Chassis()
        chassis.ParseFromString(msg)
        timestamp = chassis.header.timestamp_sec
        if self._current_driving_mode == chassis.driving_mode:
            # DrivingMode doesn't change.
            return
        # Save disengagement.
        if (self._current_driving_mode == Chassis.COMPLETE_AUTO_DRIVE and
            chassis.driving_mode == Chassis.EMERGENCY_MODE):
            glog.info('Disengagement found at', timestamp)
            disengagement = self.record.disengagements.add(time=timestamp)
            if self._last_position is not None:
                lat, lon = utm.to_latlon(self._last_position.x,
                                         self._last_position.y,
                                         gflags.FLAGS.utm_zone_id,
                                         gflags.FLAGS.utm_zone_letter)
                disengagement.location.lat = lat
                disengagement.location.lon = lon
        # Update DrivingMode.
        self._current_driving_mode = chassis.driving_mode

    def ProcessLocalization(self, msg):
        """Process Localization, stat mileages and save driving path."""
        localization = LocalizationEstimate()
        localization.ParseFromString(msg)
        timestamp = localization.header.timestamp_sec
        cur_pos = localization.pose.position

        # Stat mileages.
        if (self._last_position is not None and
            self._current_driving_mode is not None):
            driving_mode = Chassis.DrivingMode.Name(self._current_driving_mode)
            meters = utm_distance_meters(self._last_position, cur_pos)
            if driving_mode in self.record.stat.mileages:
                self.record.stat.mileages[driving_mode] += meters
            else:
                self.record.stat.mileages[driving_mode] = meters

        # Sample driving path.
        G = gflags.FLAGS
        if (self._last_position_sampled is None or
            (timestamp - self._last_position_sampled_time >
                 G.pos_sample_min_duration and
             utm_distance_meters(self._last_position_sampled, cur_pos) >
                 G.pos_sample_min_distance)):
            self._last_position_sampled = cur_pos
            self._last_position_sampled_time = timestamp
            lat, lon = utm.to_latlon(cur_pos.x, cur_pos.y,
                                     G.utm_zone_id, G.utm_zone_letter)
            self.record.stat.driving_path.add(lat=lat, lon=lon)
        # Update position.
        self._last_position = cur_pos


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    if len(sys.argv) > 0:
        print RecordParser.Parse(sys.argv[-1])
