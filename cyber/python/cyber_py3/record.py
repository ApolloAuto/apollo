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
"""Module for wrapper cyber record."""

import collections
import importlib
import os
import sys

from google.protobuf.descriptor_pb2 import FileDescriptorProto


# init vars
CYBER_PATH = os.environ['CYBER_PATH']
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER_RECORD = importlib.import_module('_cyber_record_py3')
PyBagMessage = collections.namedtuple('PyBagMessage',
                                      'topic message data_type timestamp')


class RecordReader(object):

    """
    Class for cyber RecordReader wrapper.
    """

    ##
    # @brief the constructor function.
    #
    # @param file_name the record file name.
    def __init__(self, file_name):
        self.record_reader = _CYBER_RECORD.new_PyRecordReader(file_name)

    def __del__(self):
        _CYBER_RECORD.delete_PyRecordReader(self.record_reader)

    ##
    # @brief Read message from bag file.
    #
    # @param start_time the start time to read.
    # @param end_time the end time to read.
    #
    # @return return (channnel, data, data_type, timestamp)
    def read_messages(self, start_time=0, end_time=18446744073709551615):
        while True:
            message = _CYBER_RECORD.PyRecordReader_ReadMessage(
                self.record_reader, start_time, end_time)

            if not message["end"]:
                yield PyBagMessage(message["channel_name"], message["data"],
                                   message["data_type"], message["timestamp"])
            else:
                # print "No message more."
                break

    ##
    # @brief Return message count of the channel in current record file.
    #
    # @param channel_name the channel name.
    #
    # @return return the message count.
    def get_messagenumber(self, channel_name):
        return _CYBER_RECORD.PyRecordReader_GetMessageNumber(
            self.record_reader, channel_name)

    ##
    # @brief Get the corresponding message type of channel.
    #
    # @param channel_name channel name.
    #
    # @return return the name of ther string type.
    def get_messagetype(self, channel_name):
        return _CYBER_RECORD.PyRecordReader_GetMessageType(
            self.record_reader, channel_name).decode('utf-8')

    def get_protodesc(self, channel_name):
        """
        Return message protodesc.
        """
        return _CYBER_RECORD.PyRecordReader_GetProtoDesc(
            self.record_reader, channel_name)

    def get_headerstring(self):
        """
        Return message header string.
        """
        return _CYBER_RECORD.PyRecordReader_GetHeaderString(self.record_reader)

    def reset(self):
        """
        Return reset.
        """
        return _CYBER_RECORD.PyRecordReader_Reset(self.record_reader)

    def get_channellist(self):
        """
        Return current channel names list.
        """
        return _CYBER_RECORD.PyRecordReader_GetChannelList(self.record_reader)


class RecordWriter(object):

    """
    Class for cyber RecordWriter wrapper.
    """

    ##
    # @brief the constructor function.
    #
    # @param file_segmentation_size_kb size to segment the file, 0 is no segmentation.
    # @param file_segmentation_interval_sec size to segment the file, 0 is no segmentation.
    def __init__(self, file_segmentation_size_kb=0,
                 file_segmentation_interval_sec=0):
        self.record_writer = _CYBER_RECORD.new_PyRecordWriter()
        _CYBER_RECORD.PyRecordWriter_SetSizeOfFileSegmentation(
            self.record_writer, file_segmentation_size_kb)
        _CYBER_RECORD.PyRecordWriter_SetIntervalOfFileSegmentation(
            self.record_writer, file_segmentation_interval_sec)

    def __del__(self):
        _CYBER_RECORD.delete_PyRecordWriter(self.record_writer)

    ##
    # @brief Open record file for write.
    #
    # @param path the file path.
    #
    # @return Success is True, other False.
    def open(self, path):
        return _CYBER_RECORD.PyRecordWriter_Open(self.record_writer, path)

    ##
    # @brief Close record file.
    def close(self):
        """
        Close record file.
        """
        _CYBER_RECORD.PyRecordWriter_Close(self.record_writer)

    ##
    # @brief Writer channel by channelname, typename, protodesc.
    #
    # @param channel_name the channel name to write
    # @param type_name a string of message type name.
    # @param proto_desc the message descriptor.
    #
    # @return Success is True, other False.
    def write_channel(self, channel_name, type_name, proto_desc):
        """
        Writer channel by channelname,typename,protodesc
        """
        return _CYBER_RECORD.PyRecordWriter_WriteChannel(
            self.record_writer, channel_name, type_name, proto_desc)

    ##
    # @brief Writer msg: channelname, data, writer time.
    #
    # @param channel_name channel name to write.
    # @param data when raw is True, data processed as a rawdata, other it needs to SerializeToString
    # @param time message time.
    # @param raw the flag implies data whether or not a rawdata.
    #
    # @return Success is True, other False.
    def write_message(self, channel_name, data, time, raw=True):
        """
        Writer msg:channelname,rawmsg,writer time
        """
        if raw:
            return _CYBER_RECORD.PyRecordWriter_WriteMessage(
                self.record_writer, channel_name, data, time, "")

        file_desc = data.DESCRIPTOR.file
        proto = FileDescriptorProto()
        file_desc.CopyToProto(proto)
        proto.name = file_desc.name
        desc_str = proto.SerializeToString()
        return _CYBER_RECORD.PyRecordWriter_WriteMessage(
            self.record_writer,
            channel_name, data.SerializeToString(), time, desc_str)

    def set_size_fileseg(self, size_kilobytes):
        """
        Return filesegment size.
        """
        return _CYBER_RECORD.PyRecordWriter_SetSizeOfFileSegmentation(
            self.record_writer, size_kilobytes)

    def set_intervaltime_fileseg(self, time_sec):
        """
        Return file interval time.
        """
        return _CYBER_RECORD.PyRecordWriter_SetIntervalOfFileSegmentation(
            self.record_writer, time_sec)

    def get_messagenumber(self, channel_name):
        """
        Return message count.
        """
        return _CYBER_RECORD.PyRecordWriter_GetMessageNumber(
            self.record_writer, channel_name)

    def get_messagetype(self, channel_name):
        """
        Return message type.
        """
        return _CYBER_RECORD.PyRecordWriter_GetMessageType(
            self.record_writer, channel_name).decode('utf-8')

    def get_protodesc(self, channel_name):
        """
        Return message protodesc.
        """
        return _CYBER_RECORD.PyRecordWriter_GetProtoDesc(
            self.record_writer, channel_name)
