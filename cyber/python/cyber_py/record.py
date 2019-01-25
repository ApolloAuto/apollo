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
"""Module for wrapper cyber record."""

import sys
import os
import importlib
import collections
from google.protobuf.descriptor_pb2 import FileDescriptorProto

# init vars
CYBER_PATH = os.environ['CYBER_PATH']
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")
sys.path.append(CYBER_PATH + "/python/cyber")
sys.path.append(CYBER_PATH + "/python/cyber_py")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER_RECORD = importlib.import_module('_cyber_record')
PyBagMessage = collections.namedtuple('PyBagMessage',
                                      'topic message data_type timestamp')
#//////////////////////////////record file class//////////////////////////////
class RecordReader(object):
    """
    Class for cyber RecordReader wrapper.
    """
    def __init__(self, file_name):
        self.record_reader = _CYBER_RECORD.new_PyRecordReader(file_name)

    def __del__(self):
        _CYBER_RECORD.delete_PyRecordReader(self.record_reader)

    def read_messages(self, start_time=0, end_time=18446744073709551615):
        """
        read message from bag file.
        @param self
        @param start_time:
        @param end_time:
        @return: generator of (message, data_type, timestamp)
        """
        while True:
            message = _CYBER_RECORD.PyRecordReader_ReadMessage(
                  self.record_reader, start_time, end_time)

            if not message["end"]:
                yield PyBagMessage(message["channel_name"], message["data"],
                        message["data_type"], message["timestamp"])
            else:
                #print "No message more."
                break

    def get_messagenumber(self, channel_name):
        """
        return message count.
        """
        return _CYBER_RECORD.PyRecordReader_GetMessageNumber(
                    self.record_reader, channel_name)

    def get_messagetype(self, channel_name):
        """
        return message type.
        """
        return _CYBER_RECORD.PyRecordReader_GetMessageType(
                    self.record_reader, channel_name)

    def get_protodesc(self, channel_name):
        """
        return message protodesc.
        """
        return _CYBER_RECORD.PyRecordReader_GetProtoDesc(
                    self.record_reader, channel_name)

    def get_headerstring(self):
        """
        return message header string.
        """
        return _CYBER_RECORD.PyRecordReader_GetHeaderString(self.record_reader)

    def reset(self):
        """
        return reset.
        """
        return _CYBER_RECORD.PyRecordReader_Reset(self.record_reader)

    def get_channellist(self):
        """
        return channel list.
        """
        return _CYBER_RECORD.PyRecordReader_GetChannelList(self.record_reader)

class RecordWriter(object):
    """
    Class for cyber RecordWriter wrapper.
    """
    def __init__(self):
        self.record_writer = _CYBER_RECORD.new_PyRecordWriter()

    def __del__(self):
        _CYBER_RECORD.delete_PyRecordWriter(self.record_writer)

    def open(self, path):
        """
        open record file for write.
        """
        return _CYBER_RECORD.PyRecordWriter_Open(self.record_writer, path)

    def close(self):
        """
        close record file.
        """
        _CYBER_RECORD.PyRecordWriter_Close(self.record_writer)

    def write_channel(self, channel_name, type_name, proto_desc):
        """
        writer channel by channelname,typename,protodesc
        """
        return _CYBER_RECORD.PyRecordWriter_WriteChannel(self.record_writer,
                    channel_name, type_name, proto_desc)

    def write_message(self, channel_name, data, time, raw=True):
        """
        writer msg:channelname,rawmsg,writer time
        """
        if raw:
            return _CYBER_RECORD.PyRecordWriter_WriteMessage(self.record_writer,
                    channel_name, data, time, "")
        else:
            file_desc = data.DESCRIPTOR.file
            proto = FileDescriptorProto()
            file_desc.CopyToProto(proto)
            proto.name = file_desc.name
            desc_str = proto.SerializeToString()
            return _CYBER_RECORD.PyRecordWriter_WriteMessage(self.record_writer,
                    channel_name, data.SerializeToString(), time, desc_str)

    def set_size_fileseg(self, size_kilobytes):
        """
        return filesegment size.
        """
        return _CYBER_RECORD.PyRecordWriter_SetSizeOfFileSegmentation(
                    self.record_writer, size_kilobytes)

    def set_intervaltime_fileseg(self, time_sec):
        """
        return file interval time.
        """
        return _CYBER_RECORD.PyRecordWriter_SetIntervalOfFileSegmentation(
                    self.record_writer, time_sec)

    def get_messagenumber(self, channel_name):
        """
        return message count.
        """
        return _CYBER_RECORD.PyRecordWriter_GetMessageNumber(
                    self.record_writer, channel_name)

    def get_messagetype(self, channel_name):
        """
        return message type.
        """
        return _CYBER_RECORD.PyRecordWriter_GetMessageType(
                    self.record_writer, channel_name)

    def get_protodesc(self, channel_name):
        """
        return message protodesc.
        """
        return _CYBER_RECORD.PyRecordWriter_GetProtoDesc(
                    self.record_writer, channel_name)
