
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
"""Module for example of listener."""

import time

from cyber_py import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark


def test_client_class():
    """
    Client send request
    """
    node = cyber.Node("client_node")
    client = node.create_client("server_01", ChatterBenchmark, ChatterBenchmark)
    req = ChatterBenchmark()
    req.content = "clt:Hello service!"
    req.seq = 0
    count = 0
    while not cyber.is_shutdown():
        time.sleep(1)
        count += 1
        req.seq = count
        print("-" * 80)
        response = client.send_request(req)
        print("get Response [ ", response, " ]")


if __name__ == '__main__':
    cyber.init()
    test_client_class()
    cyber.shutdown()
