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
"""Records utils."""

from modules.data.proto.record_pb2 import Record


def CombineRecords(records):
    """Combine multiple records info to one."""
    records.sort(key=lambda record: record.header.begin_time)
    virtual_record = Record(path=records[0].dir, dir=records[0].dir)
    virtual_record.header.begin_time = records[0].header.begin_time
    virtual_record.header.end_time = records[-1].header.end_time
    for record in records:
        virtual_record.header.size += record.header.size
        channels = virtual_record.channels
        for channel, count in record.channels.iteritems():
            channels[channel] = (channels.get(channel) or 0) + count

        if record.hmi_status.current_mode:
            virtual_record.hmi_status.CopyFrom(record.hmi_status)

        virtual_record.disengagements.extend(record.disengagements)
        virtual_record.drive_events.extend(record.drive_events)
        mileages = virtual_record.stat.mileages
        for driving_mode, miles in record.stat.mileages.iteritems():
            mileages[driving_mode] = (mileages.get(driving_mode) or 0) + miles
        virtual_record.stat.driving_path.extend(record.stat.driving_path)
    return virtual_record
