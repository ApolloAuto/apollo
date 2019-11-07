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
"""Module for test node."""

import sys
import unittest

sys.path.append("../")
from cyber_py import cyber
from modules.common.util.testdata.simple_pb2 import SimpleMessage


def callback(data):
    """
    Reader callback.
    """
    print "=" * 80
    print "py:reader callback msg->:"
    print data
    print "=" * 80


class TestNode(unittest.TestCase):

    """
    Class for node unit test.
    """
    @classmethod
    def setUpClass(cls):
        cyber.init()

    @classmethod
    def tearDownClass(cls):
        cyber.shutdown()

    def test_writer(self):
        """
        Unit test of writer.
        """
        msg = SimpleMessage()
        msg.text = "talker:send Alex!"
        msg.integer = 0

        self.assertTrue(cyber.ok())
        test_node = cyber.Node("node_name1")
        writer = test_node.create_writer("channel/chatter", SimpleMessage, 7)
        self.assertEqual(writer.name, "channel/chatter")
        self.assertEqual(
            writer.data_type, "apollo.common.util.test.SimpleMessage")
        self.assertTrue(writer.write(msg))

    def test_reader(self):
        """
        Unit test of reader.
        """
        self.assertTrue(cyber.ok())
        test_node = cyber.Node("listener")
        reader = test_node.create_reader("channel/chatter",
                                         SimpleMessage, callback)
        self.assertEqual(reader.name, "channel/chatter")
        self.assertEqual(reader.data_type, SimpleMessage)
        self.assertEqual(SimpleMessage.DESCRIPTOR.full_name,
                         "apollo.common.util.test.SimpleMessage")

if __name__ == '__main__':
    unittest.main()
