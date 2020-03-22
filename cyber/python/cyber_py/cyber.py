#!/usr/bin/env python2

# ****************************************************************************
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
"""Module for init environment."""

import ctypes
import importlib
import os
import sys
import threading
import time

from google.protobuf.descriptor_pb2 import FileDescriptorProto


PY_CALLBACK_TYPE = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
PY_CALLBACK_TYPE_T = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

# init vars
CYBER_PATH = os.environ.get('CYBER_PATH', '/apollo/cyber')
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")

sys.path.append(CYBER_PATH + "/lib/python/")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER = importlib.import_module('_cyber')


##
# @brief init cyber environment.
# @param module_name Used as the log file name.
#
# @return Success is True, otherwise False.
def init(module_name="cyber_py"):
    """
    init cyber environment.
    """
    return _CYBER.py_init(module_name)


def ok():
    """
    is cyber envi ok.
    """
    return _CYBER.py_ok()


def shutdown():
    """
    shutdown cyber envi.
    """
    return _CYBER.py_shutdown()


def is_shutdown():
    """
    is cyber shutdown.
    """
    return _CYBER.py_is_shutdown()


def waitforshutdown():
    """
    wait until the cyber is shutdown.
    """
    return _CYBER.py_waitforshutdown()

# //////////////////////////////class//////////////////////////////


class Writer(object):

    """
    Class for cyber writer wrapper.
    """

    def __init__(self, name, writer, data_type):
        self.name = name
        self.writer = writer
        self.data_type = data_type

    ##
    # @brief write message.
    #
    # @param data is a message type.
    #
    # @return Success is 0, otherwise False.
    def write(self, data):
        """
        writer message string
        """
        return _CYBER.PyWriter_write(self.writer, data.SerializeToString())


class Reader(object):

    """
    Class for cyber reader wrapper.
    """

    def __init__(self, name, reader, data_type):
        self.name = name
        self.reader = reader
        self.data_type = data_type


class Client(object):

    """
    Class for cyber service client wrapper.
    """

    def __init__(self, client, data_type):
        self.client = client
        self.data_type = data_type

    ##
    # @brief send request message to service.
    #
    # @param data is a message type.
    #
    # @return None or response from service.
    def send_request(self, data):
        """
        send request to service
        """
        response_str = _CYBER.PyClient_send_request(
            self.client, data.SerializeToString())
        if len(response_str) == 0:
            return None

        response = self.data_type()
        response.ParseFromString(response_str)
        return response


class Node(object):

    """
    Class for cyber Node wrapper.
    """

    def __init__(self, name):
        self.node = _CYBER.new_PyNode(name)
        self.list_writer = []
        self.list_reader = []
        self.subs = {}
        self.pubs = {}
        self.list_client = []
        self.list_service = []
        self.mutex = threading.Lock()
        self.callbacks = {}
        self.services = {}

    def __del__(self):
        # print("+++ node __del___")
        for writer in self.list_writer:
            _CYBER.delete_PyWriter(writer)
        for reader in self.list_reader:
            _CYBER.delete_PyReader(reader)
        for c in self.list_client:
            _CYBER.delete_PyClient(c)
        for s in self.list_service:
            _CYBER.delete_PyService(s)
        _CYBER.delete_PyNode(self.node)

    ##
    # @brief register proto message by proto descriptor file.
    #
    # @param file_desc object about datatype.DESCRIPTOR.file .
    def register_message(self, file_desc):
        """
        register proto message desc file.
        """
        for dep in file_desc.dependencies:
            self.register_message(dep)
        proto = FileDescriptorProto()
        file_desc.CopyToProto(proto)
        proto.name = file_desc.name
        desc_str = proto.SerializeToString()
        _CYBER.PyNode_register_message(self.node, desc_str)

    ##
    # @brief create a channel writer for send message to another channel.
    #
    # @param name is the channel name.
    # @param data_type is message class for serialization
    # @param qos_depth is a queue size, which defines the size of the cache.
    #
    # @return return the writer object.
    def create_writer(self, name, data_type, qos_depth=1):
        """
        create a channel writer for send message to another channel.
        """
        self.register_message(data_type.DESCRIPTOR.file)
        datatype = data_type.DESCRIPTOR.full_name
        writer = _CYBER.PyNode_create_writer(self.node, name,
                                                  datatype, qos_depth)
        self.list_writer.append(writer)
        return Writer(name, writer, datatype)

    def reader_callback(self, name):
        sub = self.subs[name]
        msg_str = _CYBER.PyReader_read(sub[0], False)
        if len(msg_str) > 0:
            if sub[3] != "RawData":
                proto = sub[3]()
                proto.ParseFromString(msg_str)
            else:
                # print "read rawdata-> ",sub[3]
                proto = msg_str

            if sub[2] is None:
                sub[1](proto)
            else:
                sub[1](proto, sub[2])
        return 0

    ##
    # @brief create a channel reader for receive message from another channel.
    #
    # @param name the channel name to read.
    # @param data_type  message class for serialization
    # @param callback function to call (fn(data)) when data is received. If
    # args is set, the function must accept the args as a second argument,
    # i.e. fn(data, args)
    # @param args additional arguments to pass to the callback
    #
    # @return return the writer object.
    def create_reader(self, name, data_type, callback, args=None):
        """
        create a channel reader for receive message from another channel.
        """
        self.mutex.acquire()
        if name in self.subs.keys():
            self.mutex.release()
            return None
        self.mutex.release()

        # datatype = data_type.DESCRIPTOR.full_name
        reader = _CYBER.PyNode_create_reader(
            self.node, name, str(data_type))
        if reader is None:
            return None
        self.list_reader.append(reader)
        sub = (reader, callback, args, data_type, False)

        self.mutex.acquire()
        self.subs[name] = sub
        self.mutex.release()
        fun_reader_cb = PY_CALLBACK_TYPE(self.reader_callback)
        self.callbacks[name] = fun_reader_cb
        f_ptr = ctypes.cast(self.callbacks[name], ctypes.c_void_p).value
        _CYBER.PyReader_register_func(reader, f_ptr)

        return Reader(name, reader, data_type)

    def create_rawdata_reader(self, name, callback, args=None):
        """
        Create RawData reader:listener RawMessage
        """
        return self.create_reader(name, "RawData", callback, args)

    ##
    # @brief create client for the c/s.
    #
    # @param name the service name.
    # @param request_data_type the request message type.
    # @param response_data_type the response message type.
    #
    # @return the client object.
    def create_client(self, name, request_data_type, response_data_type):
        datatype = request_data_type.DESCRIPTOR.full_name
        c = _CYBER.PyNode_create_client(self.node, name,
                                             str(datatype))
        self.list_client.append(c)
        return Client(c, response_data_type)

    def service_callback(self, name):
        v = self.services[name]
        msg_str = _CYBER.PyService_read(v[0])
        if (len(msg_str) > 0):
            proto = v[3]()
            proto.ParseFromString(msg_str)
            response = None
            if v[2] is None:
                response = v[1](proto)
            else:
                response = v[1](proto, v[2])
            _CYBER.PyService_write(v[0], response.SerializeToString())
        return 0

    ##
    # @brief create client for the c/s.
    #
    # @param name the service name.
    # @param req_data_type the request message type.
    # @param res_data_type the response message type.
    # @param callback function to call (fn(data)) when data is received. If
    # args is set, the function must accept the args as a second argument,
    # i.e. fn(data, args)
    # @param args additional arguments to pass to the callback.
    #
    # @return return the service object.
    def create_service(self, name, req_data_type, res_data_type, callback,
                       args=None):
        self.mutex.acquire()
        if name in self.services.keys():
            self.mutex.release()
            return None
        self.mutex.release()
        datatype = req_data_type.DESCRIPTOR.full_name
        s = _CYBER.PyNode_create_service(self.node, name, str(datatype))
        self.list_service.append(s)
        v = (s, callback, args, req_data_type, False)
        self.mutex.acquire()
        self.services[name] = v
        self.mutex.release()
        f = PY_CALLBACK_TYPE(self.service_callback)
        self.callbacks[name] = f
        f_ptr = ctypes.cast(f, ctypes.c_void_p).value
        _CYBER.PyService_register_func(s, f_ptr)
        return s

    def spin(self):
        """
        spin for every 0.002s.
        """
        while not _CYBER.py_is_shutdown():
            time.sleep(0.002)


class ChannelUtils(object):

    @staticmethod
    ##
    # @brief Parse rawmsg from rawmsg data by message type.
    #
    # @param msg_type message type.
    # @param rawmsgdata rawmsg data.
    #
    # @return a human readable form of this message. For debugging and
    # other purposes.
    def get_debugstring_rawmsgdata(msg_type, rawmsgdata):
        return _CYBER.PyChannelUtils_get_debugstring_by_msgtype_rawmsgdata(msg_type, rawmsgdata)

    @staticmethod
    ##
    # @brief Parse rawmsg from channel name.
    #
    # @param channel_name channel name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return return the messsage type of this channel.
    def get_msgtype(channel_name, sleep_s=2):
        return _CYBER.PyChannelUtils_get_msg_type(channel_name, sleep_s)

    @staticmethod
    ##
    # @brief Get all active channel names
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active channel names.
    def get_channels(sleep_s=2):
        return _CYBER.PyChannelUtils_get_active_channels(sleep_s)

    @staticmethod
    ##
    # @brief Get the active channel info.
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active channels info. {'channel1':[], 'channel2':[]} .
    def get_channels_info(sleep_s=2):
        return _CYBER.PyChannelUtils_get_channels_info(sleep_s)


class NodeUtils(object):

    @staticmethod
    ##
    # @brief Get all active node names.
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active node names.
    def get_nodes(sleep_s=2):
        return _CYBER.PyNodeUtils_get_active_nodes(sleep_s)

    @staticmethod
    ##
    # @brief Get node attribute by the node name.
    #
    # @param node_name node name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return the node's attribute.
    def get_node_attr(node_name, sleep_s=2):
        return _CYBER.PyNodeUtils_get_node_attr(node_name, sleep_s)

    @staticmethod
    ##
    # @brief Get node's reader channel names
    #
    # @param node_name the node name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return node's reader channel names.
    def get_readersofnode(node_name, sleep_s=2):
        return _CYBER.PyNodeUtils_get_readersofnode(node_name, sleep_s)

    @staticmethod
    ##
    # @brief Get node's writer channel names.
    #
    # @param node_name the node name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return node's writer channel names.
    def get_writersofnode(node_name, sleep_s=2):
        return _CYBER.PyNodeUtils_get_writersofnode(node_name, sleep_s)


class ServiceUtils(object):

    @staticmethod
    ##
    # @brief Get all active service names.
    #
    # @param sleep_s wait time for topo discovery.
    #
    # @return all active service names.
    def get_services(sleep_s=2):
        return _CYBER.PyServiceUtils_get_active_services(sleep_s)

    @staticmethod
    ##
    # @brief Get service attribute by the service name.
    #
    # @param service_name service name.
    # @param sleep_s wait time for topo discovery.
    #
    # @return the service's attribute.
    def get_service_attr(service_name, sleep_s=2):
        return _CYBER.PyServiceUtils_get_service_attr(service_name, sleep_s)
