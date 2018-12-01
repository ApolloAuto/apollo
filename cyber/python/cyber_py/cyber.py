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
"""Module for init environment."""

import sys
import os
import importlib
import time
import threading
import ctypes

from google.protobuf.descriptor_pb2 import FileDescriptorProto

PY_CALLBACK_TYPE = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
PY_CALLBACK_TYPE_T = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

# init vars
CYBER_PATH = os.environ['CYBER_PATH']
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")
sys.path.append(CYBER_PATH + "/python/cyber")
sys.path.append(CYBER_PATH + "/python/cyber_py")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER_INIT = importlib.import_module('_cyber_init')
_CYBER_NODE = importlib.import_module('_cyber_node')


def init(module_name="cyber_py"):
    """
    init cyber.
    """
    return _CYBER_INIT.py_init(module_name)


def ok():
    """
    is cyber envi ok.
    """
    return _CYBER_INIT.py_ok()


def shutdown():
    """
    shutdown cyber envi.
    """
    return _CYBER_INIT.py_shutdown()


def is_shutdown():
    """
    is cyber shutdown.
    """
    return _CYBER_INIT.py_is_shutdown()


def waitforshutdown():
    """
    waitforshutdown.
    """
    return _CYBER_INIT.py_waitforshutdown()

# //////////////////////////////class//////////////////////////////

class Writer(object):
    """
    Class for cyber writer wrapper.
    """

    def __init__(self, name, writer, data_type):
        self.name = name
        self.writer = writer
        self.data_type = data_type

    def write(self, data):
        """
        writer msg string
        """
        return _CYBER_NODE.PyWriter_write(self.writer, data.SerializeToString())


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

    def send_request(self, data):
        """
        send request to service
        @param self
        @param data: proto message to send
        @return : None or response
        """
        response_str = _CYBER_NODE.PyClient_send_request(
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
        self.node = _CYBER_NODE.new_PyNode(name)
        self.list_writer = []
        self.list_reader = []
        self.subs = {}
        self.pubs = {}
        self.mutex = threading.Lock()
        self.callbacks = {}

    def __del__(self):
        #print("+++ node __del___")
        for writer in self.list_writer:
            _CYBER_NODE.delete_PyWriter(writer)
        for reader in self.list_reader:
            _CYBER_NODE.delete_PyReader(reader)
        _CYBER_NODE.delete_PyNode(self.node)

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
        _CYBER_NODE.PyNode_register_message(self.node, desc_str)

    def create_writer(self, name, data_type, qos_depth=1):
        """
        create a topic writer for send message to topic.
        @param self
        @param name str: topic name
        @param data_type proto: message class for serialization
        """
        self.register_message(data_type.DESCRIPTOR.file)
        datatype = data_type.DESCRIPTOR.full_name
        writer = _CYBER_NODE.PyNode_create_writer(self.node, name,
            datatype, qos_depth)
        self.list_writer.append(writer)
        return Writer(name, writer, datatype)

    def reader_callback(self, name):
        """
        reader callback
        """
        sub = self.subs[name]
        msg_str = _CYBER_NODE.PyReader_read(sub[0], False)
        if len(msg_str) > 0:
            proto = sub[3]()
            proto.ParseFromString(msg_str)
            # response = None
            if sub[2] is None:
                sub[1](proto)
            else:
                sub[1](proto, sub[2])
        return 0

    def create_reader(self, name, data_type, callback, args=None):
        """
        create a topic reader for receive message from topic.
        @param self
        @param name str: topic name
        @param data_type proto: message class for serialization
        @callback fn: function to call (fn(data)) when data is
                   received. If args is set, the function must
                   accept the args as a second argument,
                   i.e. fn(data, args)
        @args any: additional arguments to pass to the callback
        """
        self.mutex.acquire()
        if name in self.subs.keys():
            self.mutex.release()
            return None
        self.mutex.release()

        # datatype = data_type.DESCRIPTOR.full_name
        reader = _CYBER_NODE.PyNode_create_reader(
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
        _CYBER_NODE.PyReader_register_func(reader, f_ptr)

        return Reader(name, reader, data_type)

    def spin(self):
        """
        spin in wait and process message.
        @param self
        """
        while not _CYBER_INIT.py_is_shutdown():
            time.sleep(0.002)
            self.do_executable()

    def do_executable(self):
        """
        process received message.
        @param self
        """
        self.mutex.acquire()
        for _, item in self.subs.items():
            msg_str = _CYBER_NODE.PyReader_read(item[0], False)
            if len(msg_str) > 0:
                if item[4]:
                    if item[2] is None:
                        item[1](msg_str)
                    else:
                        item[1](msg_str, item[2])
                else:
                    proto = item[3]()
                    proto.ParseFromString(msg_str)
                    if item[2] is None:
                        item[1](proto)
                    else:
                        item[1](proto, item[2])
        self.mutex.release()
