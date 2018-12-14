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
"""
dump road test log.
Usage:
    ./dump_road_test_log.py bag1 bag2 ...
"""
import sys
import time

from cyber_py import cyber
from cyber_py.record import RecordReader
from modules.common.proto import drive_event_pb2


kEventTopic = '/apollo/drive_event'


class EventDumper(object):
    """Dump event."""

    def __init__(self):
        """Init."""

    def calculate(self, bag_file):
        """Calculate mileage."""
        try:
	    drive_event = drive_event_pb2.DriveEvent()
            reader = RecordReader(bag_file)
        except:
            print("can't open bag")
        else:
            f = file('/apollo/test.txt', 'a')
            for msg in reader.read_messages():
                if msg.topic == kEventTopic:
                    drive_event.ParseFromString(msg.message)
                    msg_time = time.localtime(msg.timestamp/long(1e9))
                    f.write(time.strftime("%Y-%m-%d %H:%M:%S", msg_time))
                    f.write(str(drive_event.type)+":")
                    f.write(drive_event.event.encode('utf-8')+'\n')
            f.close()


def main():
    """Main function."""

    ed = EventDumper()
    for bag_file in sys.argv[1:]:
        ed.calculate(bag_file)


if __name__ == '__main__':
    main()
