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
import importlib
import threading
import ctypes

PY_CALLBACK_TYPE = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
PY_CALLBACK_TYPE_T = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)

# init vars
CYBERTRON_PATH = os.environ['CYBERTRON_PATH']
CYERTRON_DIR = os.path.split(CYBERTRON_PATH)[0]
sys.path.append(CYBERTRON_PATH + "/third_party/")
sys.path.append(CYBERTRON_PATH + "/lib/")
sys.path.append(CYBERTRON_PATH + "/python/cybertron")

sys.path.append(CYERTRON_DIR + "/python/")
sys.path.append(CYERTRON_DIR + "/cybertron/")

_CYBER_INIT = importlib.import_module('_cyber_init')
_CYBER_NODE = importlib.import_module('_cyber_node')


def init():
    """
    init cybertron.
    """
    return _CYBER_INIT.py_init()


def ok():
    """
    is cybertron envi ok.
    """
    return _CYBER_INIT.py_ok()


def shutdown():
    """
    shutdown cybertron envi.
    """
    return _CYBER_INIT.py_shutdown()


def is_shutdown():
    """
    is cybertron shutdown.
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
    Class for cybertron writer wrapper.
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
    Class for cybertron reader wrapper.
    """

    def __init__(self, name, reader, data_type):
        self.name = name
        self.reader = reader
        self.data_type = data_type


class Client(object):
    """
    Class for cybertron service client wrapper.
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
    Class for cybertron Node wrapper.
    """

    def __init__(self, name):
        self.node = _CYBER_NODE.new_PyNode(name)
        self.list_writer = []
        self.list_reader = []
        self.list_client = []
        self.list_service = []
        self.subs = {}
        self.pubs = {}
        self.services = {}
        self.mutex = threading.Lock()

    def __del__(self):
        #print("+++ node __del___")
        for writer in self.list_writer:
            _CYBER_NODE.delete_PyWriter(writer)
        for reader in self.list_reader:
            _CYBER_NODE.delete_PyReader(reader)
        for client in self.list_client:
            _CYBER_NODE.delete_PyClient(client)
        for service in self.list_service:
            _CYBER_NODE.delete_PyService(service)
        _CYBER_NODE.delete_PyNode(self.node)

    def create_writer(self, name, data_type):
        """
        create a topic writer for send message to topic.
        @param self
        @param name str: topic name
        @param data_type proto: message class for serialization
        """
        writer = _CYBER_NODE.PyNode_create_writer(self.node, name, data_type)
        self.list_writer.append(writer)
        return Writer(name, writer, data_type)

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
        f_ptr = ctypes.cast(fun_reader_cb, ctypes.c_void_p).value
        _CYBER_NODE.PyReader_register_func(reader, f_ptr)

        return Reader(name, reader, data_type)

    def create_client(self, name, request_data_type, response_data_type):
        """
        create client by channelname,request datatype,response datatype
        """
        datatype = request_data_type.DESCRIPTOR.full_name
        client = _CYBER_NODE.PyNode_create_client(
            self.node, name, str(datatype))
        self.list_client.append(client)
        return Client(client, response_data_type)

    def service_callback(self, name):
        """
        service callback
        """
        svr = self.services[name]
        msg_str = _CYBER_NODE.PyService_read(svr[0])

        if len(msg_str) > 0:
            proto = svr[3]()
            proto.ParseFromString(msg_str)
            response = None
            if svr[2] is None:
                response = svr[1](proto)
            else:
                response = svr[1](proto, svr[2])

            _CYBER_NODE.PyService_write(svr[0], response.SerializeToString())
        return 0

    def create_service(self, name, req_data_type, callback, args=None):
        """
        create service callback
        """
        self.mutex.acquire()
        if name in self.services.keys():
            self.mutex.release()
            return None
        self.mutex.release()

        datatype = req_data_type.DESCRIPTOR.full_name
        svr = _CYBER_NODE.PyNode_create_service(self.node, name, str(datatype))
        self.list_service.append(svr)
        srv_list = (svr, callback, args, req_data_type, False)

        self.mutex.acquire()
        self.services[name] = srv_list
        self.mutex.release()

        fun_svr_cb = PY_CALLBACK_TYPE(self.service_callback)
        f_ptr = ctypes.cast(fun_svr_cb, ctypes.c_void_p).value

        _CYBER_NODE.PyService_register_func(svr, f_ptr)
        return svr

    def spin(self):
        """
        spin in wait and process message.
        @param self
        """
        while not _CYBER_NODE.py_is_shutdown():
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
