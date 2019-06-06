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
"""Module for cyber topology manager."""

import sys
import os
import importlib
import time
import threading
import ctypes

PY_CALLBACK_TYPE = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p)
# init vars
CYBER_PATH = os.environ['CYBER_PATH']
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")
sys.path.append(CYBER_PATH + "/python/cyber")
sys.path.append(CYBER_PATH + "/python/cyber_py")
sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER_TOPOLOGY_MANAGER = importlib.import_module('_cyber_topology_manager')


class Topology_Manager(object):

    """
    Class for cyber Node wrapper.
    """

    def __init__(self):
        """
        Init Topology Manager.
        """
        print("init topology manager successful")
        self.node_manager = None
        self.channel_manager = None

    def has_node(self, node_name):
        """
        check Has Specific Node.
        """
        if not self.node_manager:
            self.node_manager = _CYBER_TOPOLOGY_MANAGER.new_Node_Manager()
        flag = _CYBER_TOPOLOGY_MANAGER.Py_HasNode(self.node_manager, node_name)
        return flag

    def get_node_list(self):
        """
        Get Node List.
        """
        if not self.node_manager:
            self.node_manager = _CYBER_TOPOLOGY_MANAGER.new_Node_Manager()
        node_list = _CYBER_TOPOLOGY_MANAGER.Py_GetNodeList(self.node_manager)
        return node_list

    def show_node_info(self, node_name):
        """
        Show Node Info.
        """
        if not self.node_manager:
            self.node_manager = _CYBER_TOPOLOGY_MANAGER.new_Node_Manager()
        ret = _CYBER_TOPOLOGY_MANAGER.Py_ShowNodeInfo(
            self.node_manager, node_name)
        return ret

    def get_channel_list(self):
        """
        Get Channel List.
        """
        if not self.channel_manager:
            self.channel_manager = _CYBER_TOPOLOGY_MANAGER.new_Channel_Manager(
            )
        channel_list = _CYBER_TOPOLOGY_MANAGER.Py_GetChannelList(
            self.channel_manager)
        return channel_list

    def get_reader_list(self):
        """
        Get Reader List.
        """
        if not self.channel_manager:
            self.channel_manager = _CYBER_TOPOLOGY_MANAGER.new_Channel_Manager(
            )
        reader_list = _CYBER_TOPOLOGY_MANAGER.Py_GetReaderList(
            self.channel_manager)
        return reader_list

    def get_writer_list(self):
        """
        Get Writer List.
        """
        if not self.channel_manager:
            self.channel_manager = _CYBER_TOPOLOGY_MANAGER.new_Channel_Manager(
            )
        writer_list = _CYBER_TOPOLOGY_MANAGER.Py_GetWriterList(
            self.channel_manager)
        return writer_list

    def get_node_writes(self, node_name):
        """
        Get Node Writes.
        """
        if not self.channel_manager:
            self.channel_manager = _CYBER_TOPOLOGY_MANAGER.new_Channel_Manager(
            )
        node_writers = _CYBER_TOPOLOGY_MANAGER.Py_GetWritersOfNode(
            self.channel_manager, node_name)
        return node_writers

    def get_node_readers(self, node_name):
        """
        Get Node Readers.
        """
        if not self.channel_manager:
            self.channel_manager = _CYBER_TOPOLOGY_MANAGER.new_Channel_Manager(
            )
        node_readers = _CYBER_TOPOLOGY_MANAGER.Py_GetReadersOfNode(
            self.channel_manager, node_name)
        return node_readers

    def show_channel_info(self, channel_name):
        """
        Show Channel Info.
        """
        if not self.channel_manager:
            self.channel_manager = _CYBER_TOPOLOGY_MANAGER.new_Channel_Manager(
            )
        ret = _CYBER_TOPOLOGY_MANAGER.Py_ShowChannelInfo(
            self.channel_manager, channel_name)
        return ret
