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
"""Module for test record."""

import unittest

from cyber.proto import record_pb2
from cyber.python.cyber_py3 import record
from cyber.proto.simple_pb2 import SimpleMessage


TEST_RECORD_FILE = "/tmp/test02.record"
CHAN_1 = "channel/chatter"
MSG_TYPE = "apollo.common.util.test.SimpleMessage"
PROTO_DESC = b"1234567890"
MSG_DATA = b"0123456789"
TIME = 999


class TestRecord(unittest.TestCase):

    """
    Class for record unit test.
    """

    def test_record_writer_read(self):
        """
        unit test of record.
        """
        # writer
        fwriter = record.RecordWriter()
        fwriter.set_size_fileseg(0)
        fwriter.set_intervaltime_fileseg(0)

        self.assertTrue(fwriter.open(TEST_RECORD_FILE))
        fwriter.write_channel(CHAN_1, MSG_TYPE, PROTO_DESC)
        fwriter.write_message(CHAN_1, MSG_DATA, TIME)

        self.assertEqual(1, fwriter.get_messagenumber(CHAN_1))
        self.assertEqual(MSG_TYPE, fwriter.get_messagetype(CHAN_1))
        self.assertEqual(PROTO_DESC, fwriter.get_protodesc(CHAN_1))
        fwriter.close()

        # reader
        fread = record.RecordReader(TEST_RECORD_FILE)
        channel_list = fread.get_channellist()
        self.assertEqual(1, len(channel_list))
        self.assertEqual(CHAN_1, channel_list[0])

        header = record_pb2.Header()
        header.ParseFromString(fread.get_headerstring())
        self.assertEqual(1, header.major_version)
        self.assertEqual(0, header.minor_version)
        self.assertEqual(1, header.chunk_number)
        self.assertEqual(1, header.channel_number)
        self.assertTrue(header.is_complete)

        for channelname, msg, datatype, timestamp in fread.read_messages():
            self.assertEqual(CHAN_1, channelname)
            self.assertEqual(MSG_DATA, msg)
            self.assertEqual(TIME, timestamp)
            self.assertEqual(1, fread.get_messagenumber(channelname))
            self.assertEqual(MSG_TYPE, datatype)
            self.assertEqual(MSG_TYPE, fread.get_messagetype(channelname))


if __name__ == '__main__':
    unittest.main()
