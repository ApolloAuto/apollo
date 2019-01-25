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
        init Node_manager.
        """
        print("init successful")
        self.node_manager = "null"

    def cyber_py_has_node(self, node_name):
        """
        check Node_manager has node.
        """
        print("node name:", node_name)
        self.node_manager = _CYBER_TOPOLOGY_MANAGER.new_Node_Manager()
        flag = _CYBER_TOPOLOGY_MANAGER.Py_HasNode(self.node_manager, node_name)
        return flag

    def cyber_py_get_nodes_name(self):
        """
        Get Node_manager nodes.
        """
        self.node_manager = _CYBER_TOPOLOGY_MANAGER.new_Node_Manager()
        nodes = _CYBER_TOPOLOGY_MANAGER.Py_GetNodesName(self.node_manager)
        if(len(nodes)):
            print("get all nodes completed")
        else:
            print("get all nodes failed")
        return nodes

    def cyber_py_show_node_info(self, node_name):
        """
        Show node Info.
        """
        self.node_manager = _CYBER_TOPOLOGY_MANAGER.new_Node_Manager()
        ret = _CYBER_TOPOLOGY_MANAGER.Py_ShowNodeInfo(self.node_manager, node_name)
        return ret

if __name__=="__main__":
     _tm = Topology_Manager()


