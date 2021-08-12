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

"""
Extract localization message from data record file,
and save them into specified  file

Usage:
    extract_path.py save_fileName  bag1 bag2

See the gflags for more optional args.
"""
import os
import sys
from cyber.python.cyber_py3.record import RecordReader
from cyber.python.cyber_py3.record import RecordWriter

input_record="/apollo/data/sensor_rgb.record"
freader = RecordReader(input_record)
fwriter = RecordWriter()
desc=None
topic_descs = {}
if not fwriter.open("/apollo/data/sensor_rgb_new.record"):
    print('writer open failed!')
    exit()
print('----- Begin to process record -----')
for channelname, msg, datatype, timestamp in freader.read_messages():
    if channelname =="/apollo/sensor/camera/traffic/image_short":
        desc = freader.get_protodesc(channelname)
        channelname="/apollo/sensor/camera/front_6mm/image"
    if channelname not in topic_descs:
            topic_descs[channelname] = freader.get_protodesc(channelname)
            fwriter.write_channel(channelname, datatype, topic_descs[channelname])
    fwriter.write_message(channelname, msg, timestamp)
print('----- Finish processing record -----')

