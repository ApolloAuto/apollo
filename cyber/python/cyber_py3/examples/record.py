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
"""Module for example of record."""

import time

from cyber.proto.unit_test_pb2 import Chatter
from cyber_py3 import record
from google.protobuf.descriptor_pb2 import FileDescriptorProto
from modules.common.util.testdata.simple_pb2 import SimpleMessage


MSG_TYPE = "apollo.common.util.test.SimpleMessage"
MSG_TYPE_CHATTER = "apollo.cyber.proto.Chatter"


def test_record_writer(writer_path):
    """
    Record writer.
    """
    fwriter = record.RecordWriter()
    fwriter.set_size_fileseg(0)
    fwriter.set_intervaltime_fileseg(0)

    if not fwriter.open(writer_path):
        print('Failed to open record writer!')
        return
    print('+++ Begin to writer +++')

    # Writer 2 SimpleMessage
    msg = SimpleMessage()
    msg.text = "AAAAAA"

    file_desc = msg.DESCRIPTOR.file
    proto = FileDescriptorProto()
    file_desc.CopyToProto(proto)
    proto.name = file_desc.name
    desc_str = proto.SerializeToString()
    print(msg.DESCRIPTOR.full_name)
    fwriter.write_channel(
        'simplemsg_channel', msg.DESCRIPTOR.full_name, desc_str)
    fwriter.write_message('simplemsg_channel', msg, 990, False)
    fwriter.write_message('simplemsg_channel', msg.SerializeToString(), 991)

    # Writer 2 Chatter
    msg = Chatter()
    msg.timestamp = 99999
    msg.lidar_timestamp = 100
    msg.seq = 1

    file_desc = msg.DESCRIPTOR.file
    proto = FileDescriptorProto()
    file_desc.CopyToProto(proto)
    proto.name = file_desc.name
    desc_str = proto.SerializeToString()
    print(msg.DESCRIPTOR.full_name)
    fwriter.write_channel('chatter_a', msg.DESCRIPTOR.full_name, desc_str)
    fwriter.write_message('chatter_a', msg, 992, False)
    msg.seq = 2
    fwriter.write_message("chatter_a", msg.SerializeToString(), 993)

    fwriter.close()


def test_record_reader(reader_path):
    """
    Record reader.
    """
    freader = record.RecordReader(reader_path)
    time.sleep(1)
    print('+' * 80)
    print('+++ Begin to read +++')
    count = 0
    for channel_name, msg, datatype, timestamp in freader.read_messages():
        count += 1
        print('=' * 80)
        print('read [%d] messages' % count)
        print('channel_name -> %s' % channel_name)
        print('msgtime -> %d' % timestamp)
        print('msgnum -> %d' % freader.get_messagenumber(channel_name))
        print('msgtype -> %s' % datatype)
        print('message is -> %s' % msg)
        print('***After parse(if needed),the message is ->')
        if datatype == MSG_TYPE:
            msg_new = SimpleMessage()
            msg_new.ParseFromString(msg)
            print(msg_new)
        elif datatype == MSG_TYPE_CHATTER:
            msg_new = Chatter()
            msg_new.ParseFromString(msg)
            print(msg_new)


if __name__ == '__main__':
    test_record_file = "/tmp/test_writer.record"

    print('Begin to write record file: {}'.format(test_record_file))
    test_record_writer(test_record_file)

    print('Begin to read record file: {}'.format(test_record_file))
    test_record_reader(test_record_file)
