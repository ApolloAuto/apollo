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
"""Module for example of record."""

import time
import sys

sys.path.append("../")
from cybertron import cybertron
from cybertron import record

TEST_RECORD_FILE = "test02.record"
CHAN_1 = "channel/chatter"
CHAN_2 = "/test2"
MSG_TYPE = "apollo.cybertron.proto.Test"
STR_10B = "1234567890"
TEST_FILE = "test.record"

def test_record_writer(writer_path):
    """
    record writer.
    """
    fwriter = record.RecordWriter()
    if not fwriter.open(writer_path):
        print "writer open failed!"
        return
    print "+++ begin to writer..."
    fwriter.write_channel(CHAN_1, MSG_TYPE, STR_10B)
    fwriter.write_message(CHAN_1, STR_10B, 1000)
    fwriter.close()

def test_record_reader(reader_path):
    """
    record reader.
    """
    freader = record.RecordReader()
    if not freader.open(reader_path):
        print "reader open failed!"
        return
    time.sleep(1)
    print "+"*80
    print "+++begin to read..."
    count = 1
    read_msg_succ = True
    while not freader.endoffile():
        print "="*80
        print "read [%d] msg" % count
        read_msg_succ = freader.read_message()
        if read_msg_succ:
            channelname = freader.currentmessage_channelname()
            print "chnanel_name -> %s" % freader.currentmessage_channelname()
            print "msg -> %s" % freader.current_rawmessage()
            print "msgtime -> %d" % freader.currentmessage_time()
            print "msgnum -> %d" % freader.get_messagenumber(channelname)
            print "msgtype -> %s" % freader.get_messagetype(channelname)
            print "pbdesc -> %s" % freader.get_protodesc(channelname)
            count = count + 1
    freader.close()

    cybertron.shutdown()

if __name__ == '__main__':
    cybertron.init()
    test_record_writer(TEST_RECORD_FILE)
    test_record_reader(TEST_RECORD_FILE)
    cybertron.shutdown()
