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
"""Module for test node."""

import sys
import unittest

sys.path.append("../")
from cybertron import cybertron
from proto import chatter_pb2

def callback(data):
    """
    reader callback.
    """
    print "="*80
    print "py:reader callback msg->:"
    print data
    print "="*80

class TestNode(unittest.TestCase):
    """
    Class for node unit test.
    """
    @classmethod
    def setUpClass(cls):
        cybertron.init()

    @classmethod
    def tearDownClass(cls):
        cybertron.shutdown()

    def test_writer(self):
        """
        unit test of writer.
        """
        msg = chatter_pb2.Chatter()
        msg.content = "talker:send Alex!"
        msg.seq = 0
        msg.timestamp = 0
        msg.lidar_timestamp = 0

        self.assertTrue(cybertron.ok())
        test_node = cybertron.Node("node_name1")
        writer = test_node.create_writer("channel/chatter",
                chatter_pb2.Chatter.DESCRIPTOR.full_name)
        self.assertEqual(writer.name, "channel/chatter")
        self.assertEqual(writer.data_type, "apollo.cybertron.proto.Chatter")
        self.assertTrue(writer.write(msg))

    def test_reader(self):
        """
        unit test of reader.
        """
        self.assertTrue(cybertron.ok())
        test_node = cybertron.Node("listener")
        reader = test_node.create_reader("channel/chatter",
                chatter_pb2.Chatter, callback)
        self.assertEqual(reader.name, "channel/chatter")
        self.assertEqual(reader.data_type, chatter_pb2.Chatter)
        self.assertEqual(chatter_pb2.Chatter.DESCRIPTOR.full_name,
                "apollo.cybertron.proto.Chatter")

if __name__ == '__main__':
    unittest.main()
