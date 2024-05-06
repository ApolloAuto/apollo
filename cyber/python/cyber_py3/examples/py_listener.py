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
"""Module for example of listener."""

from cyber.python.cyber_py3 import cyber
from cyber.proto.unit_test_pb2 import ChatterBenchmark


def callback(data):
    """
    Reader message callback.
    """
    print("=" * 80)
    print("py:reader callback msg->:")
    print(data)
    print("=" * 80)


def test_listener_class():
    """
    Reader message.
    """
    print("=" * 120)
    test_node = cyber.Node("listener")
    test_node.create_reader("channel/chatter", ChatterBenchmark, callback)
    test_node.spin()


if __name__ == '__main__':
    cyber.init()
    test_listener_class()
    cyber.shutdown()
