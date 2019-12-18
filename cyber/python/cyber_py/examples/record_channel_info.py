#!/usr/bin/env python2

# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
"""Module for example of record."""

import sys

from cyber_py import cyber
from cyber_py import record
from cyber.proto import record_pb2


def print_channel_info(file_path):
    freader = record.RecordReader(file_path)
    channels = freader.get_channellist()

    header_msg = freader.get_headerstring()
    header = record_pb2.Header()
    header.ParseFromString(header_msg)

    print('\n++++++++++++Begin Channel Info Statistics++++++++++++++')
    print('-' * 40)
    print(
        'record version: [%d:%d]' %
        (header.major_version, header.minor_version))
    print('record message_number: %s' % str(header.message_number))
    print('record file size(Byte): %s' % str(header.size))
    print('chunk_number: %d' % header.chunk_number)
    print('channel count: %d' % len(channels))
    print('-' * 40)
    count = 0
    for channel in channels:
        desc = freader.get_protodesc(channel)
        count += 1
        print(
            'Channel: %s, count: %d, desc size: %d' %
            (channel, count, len(desc)))
        # print(desc)
    print("++++++++++++Finish Channel Info Statistics++++++++++++++\n")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: %s record_file' % sys.argv[0])
        sys.exit(0)

    cyber.init()
    print_channel_info(sys.argv[1])
    cyber.shutdown()
