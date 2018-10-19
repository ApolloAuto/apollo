# ****************************************************************************
# Copyright 2018 The Apollo Authors. All Rights Reserved.

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

import sys
import unittest

sys.path.append("../")
from cyber_py import cyber
from cyber_py import record
from modules.common.util.testdata.simple_pb2 import SimpleMessage

TEST_RECORD_FILE = "test02.record"
CHAN_1 = "channel/chatter"
CHAN_2 = "/test2"
MSG_TYPE = "apollo.common.util.test.SimpleMessage"
STR_10B = "1234567890"
TEST_FILE = "test.record"
TIME = 999

class TestRecord(unittest.TestCase):
    """
    Class for record unit test.
    """
    def test_record_writer_read(self):
        """
        unit test of record.
        """
        self.assertTrue(cyber.init())

        # writer
        fwriter = record.RecordWriter()
        fwriter.set_size_fileseg(200)
        fwriter.set_intervaltime_fileseg(10)

        self.assertTrue(fwriter.open(TEST_RECORD_FILE))
        fwriter.write_channel(CHAN_1, MSG_TYPE, STR_10B)
        fwriter.write_message(CHAN_1, STR_10B, TIME)

        self.assertEqual(1, fwriter.get_messagenumber(CHAN_1))
        self.assertEqual(MSG_TYPE, fwriter.get_messagetype(CHAN_1))
        self.assertEqual(STR_10B, fwriter.get_protodesc(CHAN_1))
        fwriter.close()

        # reader
        fread = record.RecordReader(TEST_RECORD_FILE)
        channel_list = fread.get_channellist()
        self.assertEqual(1, len(channel_list))
        self.assertEqual(CHAN_1, channel_list[0])
        for channelname, msg, datatype, timestamp in fread.read_messages():
            # print "+++"
            # print channelname
            # print msg, datatype, timestamp
            self.assertEqual(CHAN_1, channelname)
            self.assertEqual(STR_10B, msg)
            self.assertEqual(TIME, timestamp)
            self.assertEqual(1, fread.get_messagenumber(channelname))
            self.assertEqual(MSG_TYPE, datatype)
            self.assertEqual(MSG_TYPE, fread.get_messagetype(channelname))
            # print "pbdesc -> %s" % fread.get_protodesc(channelname)
            msg = record_pb2.Header()
            header_msg = fread.get_headerstring()
            msg.ParseFromString(header_msg)
            self.assertEqual(1, msg.major_version)
            self.assertEqual(0, msg.minor_version)
            self.assertEqual(1, msg.chunk_number)
            self.assertEqual(1, msg.channel_number)
            self.assertTrue(msg.is_complete)

        cyber.shutdown()

if __name__ == '__main__':
    unittest.main()
