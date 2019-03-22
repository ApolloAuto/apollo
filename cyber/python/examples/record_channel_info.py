# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.

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

from cyber_py import cyber
from cyber_py import record
from cyber.proto import record_pb2
from google.protobuf.descriptor_pb2 import FileDescriptorProto


def print_channel_info(file_path):
    freader = record.RecordReader(file_path)
    channels = freader.get_channellist()

    header_msg = freader.get_headerstring()
    header = record_pb2.Header()
    header.ParseFromString(header_msg)

    print ""
    print "++++++++++++Begin Channel Info Statistics++++++++++++++"
    print "-" * 40
    print "record version: %d.%d" % (header.major_version, header.minor_version)
    print "record message_number: ", header.message_number
    print "record file size(Byte) ", header.size
    print "chunk_number: ", header.chunk_number
    print "channel counts: ", len(channels)
    print "-" * 40
    counts = 1
    for channel in channels:
        desc = freader.get_protodesc(channel)
        print "[", counts, "]", "channel name: ", channel, "; desc size is ", len(desc)
        counts = counts + 1
        # print desc
    print "++++++++++++Finish Channel Info Statistics++++++++++++++"

    print ""

if __name__ == '__main__':
    cyber.init()
    rec_file = (sys.argv[1])
    print_channel_info(rec_file)
    cyber.shutdown()
