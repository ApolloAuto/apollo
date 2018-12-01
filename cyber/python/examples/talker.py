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
from modules.common.util.testdata.simple_pb2 import SimpleMessage
from cyber_py import cyber
"""Module for example of talker."""

import time
import sys

sys.path.append("../")

def test_talker_class():
    """
    test talker.
    """
    msg = SimpleMessage()
    msg.text = "talker:send Alex!"
    msg.integer = 0

    test_node = cyber.Node("node_name1")
    g_count = 1

    writer = test_node.create_writer("channel/chatter",
                                     SimpleMessage, 6)
    while not cyber.is_shutdown():
        time.sleep(1)
        g_count = g_count + 1
        msg.integer = g_count
        print "="*80
        print "write msg -> %s" % msg
        writer.write(msg)


if __name__ == '__main__':
    cyber.init("talker_sample")
    test_talker_class()
    cyber.shutdown()
