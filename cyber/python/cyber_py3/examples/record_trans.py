#!/usr/bin/env python3

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
"""Module for example of record trans."""

import sys

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import record


TEST_RECORD_FILE = "trans_ret.record"


def test_record_trans(reader_path):
    """
    Record trans.
    """
    fwriter = record.RecordWriter()
    if not fwriter.open(TEST_RECORD_FILE):
        print('Failed to open record writer!')
        return
    print('+++ Begin to trans +++')

    fread = record.RecordReader(reader_path)
    count = 0
    for channelname, msg, datatype, timestamp in fread.read_messages():
        # print channelname, timestamp, fread.get_messagenumber(channelname)
        desc = fread.get_protodesc(channelname)
        fwriter.write_channel(channelname, datatype, desc)
        fwriter.write_message(channelname, msg, timestamp)
        count += 1
    print('-' * 80)
    print('Message count: %d' % count)
    print('Channel info: ')
    channel_list = fread.get_channellist()
    print('Channel count: %d' % len(channel_list))
    print(channel_list)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: %s record_file' % sys.argv[0])
        sys.exit(0)

    cyber.init()
    test_record_trans(sys.argv[1])
    cyber.shutdown()
