#!/usr/bin/env python2

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
"""Module for test node."""

import time
import unittest

from cyber_py import cyber
from cyber_py import parameter


PARAM_SERVICE_NAME = "global_parameter_service"


class TestParams(unittest.TestCase):

    """
    Class for node unit test.
    """
    @classmethod
    def setUpClass(cls):
        cyber.init()

    @classmethod
    def tearDownClass(cls):
        cyber.shutdown()

    def test_params(self):
        param1 = parameter.Parameter("author_name", "WanderingEarth")
        param2 = parameter.Parameter("author_age", 5000)
        param3 = parameter.Parameter("author_score", 888.88)

        test_node = cyber.Node(PARAM_SERVICE_NAME)
        srv = parameter.ParameterServer(test_node)

        node_handle = cyber.Node("service_client_node")
        clt = parameter.ParameterClient(test_node, PARAM_SERVICE_NAME)
        clt.set_parameter(param1)
        clt.set_parameter(param2)
        clt.set_parameter(param3)

        param_list = clt.get_paramslist()
        self.assertEqual(3, len(param_list))
        param_list = srv.get_paramslist()
        self.assertEqual(3, len(param_list))


if __name__ == '__main__':
    unittest.main()
