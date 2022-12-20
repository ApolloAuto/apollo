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
"""Module for test cyber."""

import time
import unittest

from cyber.python.cyber_py3 import cyber
from cyber.proto.simple_pb2 import SimpleMessage


class TestCyber(unittest.TestCase):
    """
    Class for node unit test.
    """
    @staticmethod
    def callback(data):
        """
        Reader callback.
        """
        print("=" * 80)
        print("py:reader callback msg->:")
        print(data)
        print("=" * 80)

    def test_read_write(self):
        """
        Unit test of reader.
        """
        self.assertTrue(cyber.ok())
        # Read.
        reader_node = cyber.Node("listener")
        reader = reader_node.create_reader("channel/chatter",
                                           SimpleMessage, self.callback)
        self.assertEqual(reader.name, "channel/chatter")
        self.assertEqual(reader.data_type, SimpleMessage)
        self.assertEqual(SimpleMessage.DESCRIPTOR.full_name,
                         "apollo.cyber.proto.SimpleMessage")

        # Write.
        msg = SimpleMessage()
        msg.text = "talker:send Alex!"
        msg.integer = 0

        self.assertTrue(cyber.ok())
        writer_node = cyber.Node("writer")
        writer = writer_node.create_writer("channel/chatter", SimpleMessage, 7)
        self.assertEqual(writer.name, "channel/chatter")
        self.assertEqual(
            writer.data_type, "apollo.cyber.proto.SimpleMessage")
        self.assertTrue(writer.write(msg))

        # Wait for data to be processed by callback function.
        time.sleep(0.1)


if __name__ == '__main__':
    cyber.init()
    unittest.main()
    cyber.shutdown()
