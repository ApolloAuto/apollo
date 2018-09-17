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
"""Module for wrapper cybertron record."""

import sys
import os
import importlib


# init vars
CYBERTRON_PATH = os.environ['CYBERTRON_PATH']
CYERTRON_DIR = os.path.split(CYBERTRON_PATH)[0]
sys.path.append(CYBERTRON_PATH + "/third_party/")
sys.path.append(CYBERTRON_PATH + "/lib/")
sys.path.append(CYBERTRON_PATH + "/python/cybertron")

sys.path.append(CYERTRON_DIR + "/python/")
sys.path.append(CYERTRON_DIR + "/cybertron/")

_CYBER_RECORD = importlib.import_module('_cyber_record')

#//////////////////////////////record file class//////////////////////////////
class RecordReader(object):
    """
    Class for cybertron RecordReader wrapper.
    """
    def __init__(self):
        self.record_reader = _CYBER_RECORD.new_PyRecordReader()

    def __del__(self):
        _CYBER_RECORD.delete_PyRecordReader(self.record_reader)

    def open(self, path):
        """
        open record file for read.
        """
        return _CYBER_RECORD.PyRecordReader_Open(self.record_reader, path)

    def close(self):
        """
        close record file.
        """
        _CYBER_RECORD.PyRecordReader_Close(self.record_reader)

    def read_message(self):
        """
        read messge.
        """
        return _CYBER_RECORD.PyRecordReader_ReadMessage(self.record_reader)

    def endoffile(self):
        """
        is read to the end of file.
        """
        return _CYBER_RECORD.PyRecordReader_EndOfFile(self.record_reader)

    def currentmessage_channelname(self):
        """
        return current message channel name.
        """
        return _CYBER_RECORD.PyRecordReader_CurrentMessageChannelName(
                    self.record_reader)

    def current_rawmessage(self):
        """
        return current raw message.
        """
        return _CYBER_RECORD.PyRecordReader_CurrentRawMessage(
                    self.record_reader)

    def currentmessage_time(self):
        """
        return current message time.
        """
        return _CYBER_RECORD.PyRecordReader_CurrentMessageTime(
                    self.record_reader)

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

class RecordWriter(object):
    """
    Class for cybertron RecordWriter wrapper.
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

    def write_message(self, channel_name, rawmessage, time):
        """
        writer msg:channelname,rawmsg,writer time
        """
        return _CYBER_RECORD.PyRecordWriter_WriteMessage(self.record_writer,
                    channel_name, rawmessage, time)
