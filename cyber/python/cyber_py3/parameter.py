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
"""Module for init environment."""

import importlib
import os
import sys


# init vars
CYBER_PATH = os.environ.get('CYBER_PATH', '/apollo/cyber')
CYBER_DIR = os.path.split(CYBER_PATH)[0]
sys.path.append(CYBER_PATH + "/third_party/")
sys.path.append(CYBER_PATH + "/lib/")

sys.path.append(CYBER_PATH + "/lib/python/")

sys.path.append(CYBER_DIR + "/python/")
sys.path.append(CYBER_DIR + "/cyber/")

_CYBER_PARAM = importlib.import_module('_cyber_parameter_py3')


class Parameter(object):

    """
    Class for Parameter wrapper.
    """

    def __init__(self, name, value=None):
        if (name is not None and value is None):
            self.param = name
        elif (name is None and value is None):
            self.param = _CYBER_PARAM.new_PyParameter_noparam()
        elif isinstance(value, int):
            self.param = _CYBER_PARAM.new_PyParameter_int(name, value)
        elif isinstance(value, float):
            self.param = _CYBER_PARAM.new_PyParameter_double(name, value)
        elif isinstance(value, str):
            self.param = _CYBER_PARAM.new_PyParameter_string(name, value)
        else:
            print("type is not supported: ", type(value))

    def __del__(self):
        _CYBER_PARAM.delete_PyParameter(self.param)

    def type_name(self):
        """
        return Parameter typename
        """
        return _CYBER_PARAM.PyParameter_type_name(self.param)

    def descriptor(self):
        """
        return Parameter descriptor
        """
        return _CYBER_PARAM.PyParameter_descriptor(self.param)

    def name(self):
        """
        return Parameter name
        """
        return _CYBER_PARAM.PyParameter_name(self.param)

    def debug_string(self):
        """
        return Parameter debug string
        """
        return _CYBER_PARAM.PyParameter_debug_string(self.param)

    def as_string(self):
        """
        return native value
        """
        return _CYBER_PARAM.PyParameter_as_string(self.param)

    def as_double(self):
        """
        return native value
        """
        return _CYBER_PARAM.PyParameter_as_double(self.param)

    def as_int64(self):
        """
        return native value
        """
        return _CYBER_PARAM.PyParameter_as_int64(self.param)


class ParameterClient(object):

    """
    Class for ParameterClient wrapper.
    """

    ##
    # @brief constructor the ParameterClient by a node and the parameter server node name.
    #
    # @param node a node to create client.
    # @param server_node_name the parameter server's node name.
    def __init__(self, node, server_node_name):
        self.param_clt = _CYBER_PARAM.new_PyParameterClient(
            node.node, server_node_name)

    def __del__(self):
        _CYBER_PARAM.delete_PyParameterClient(self.param_clt)

    def set_parameter(self, param):
        """
        set parameter, param is Parameter.
        """
        return _CYBER_PARAM.PyParameter_clt_set_parameter(self.param_clt, param.param)

    def get_parameter(self, param_name):
        """
        get Parameter by param name param_name.
        """
        return Parameter(_CYBER_PARAM.PyParameter_clt_get_parameter(self.param_clt, param_name))

    def get_paramslist(self):
        """
        get all params of the server_node_name parameterserver.
        """
        pycapsulelist = _CYBER_PARAM.PyParameter_clt_get_parameter_list(
            self.param_clt)
        param_list = []
        for capsuleobj in pycapsulelist:
            param_list.append(Parameter(capsuleobj))
        return param_list


class ParameterServer(object):

    """
    Class for ParameterServer wrapper.
    """

    ##
    # @brief constructor the ParameterServer by the node object.
    #
    # @param node the node to support the parameter server.
    def __init__(self, node):
        self.param_srv = _CYBER_PARAM.new_PyParameterServer(node.node)

    def __del__(self):
        _CYBER_PARAM.delete_PyParameterServer(self.param_srv)

    def set_parameter(self, param):
        """
        set parameter, param is Parameter.
        """
        return _CYBER_PARAM.PyParameter_srv_set_parameter(self.param_srv, param.param)

    def get_parameter(self, param_name):
        """
        get Parameter by param name param_name.
        """
        return Parameter(_CYBER_PARAM.PyParameter_srv_get_parameter(self.param_srv, param_name))

    def get_paramslist(self):
        """
        get all params of this parameterserver.
        """
        pycapsulelist = _CYBER_PARAM.PyParameter_srv_get_parameter_list(
            self.param_srv)
        param_list = []
        for capsuleobj in pycapsulelist:
            param_list.append(Parameter(capsuleobj))
        return param_list
