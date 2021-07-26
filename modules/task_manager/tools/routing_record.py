#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
This program can replay a message pb file
"""
import os
import sys
import argparse
import glob
import time
import math
from modules.routing.proto import routing_pb2
from cyber.python.cyber_py3 import cyber
import modules.tools.common.proto_utils as proto_utils
num_=0
last_routing=None
def callback_routing(data):
    global num_
    global last_routing
    routing_=data
    frouting = open("/apollo/data/routing_.txt", 'w')
    frouting.write(str(routing_))
    frouting.close()

if __name__ == '__main__':
    cyber.init()
    node = cyber.Node("record routing file")
    routingsub = node.create_reader(
            '/apollo/routing_request', routing_pb2.RoutingRequest,
            callback_routing)
    while not cyber.is_shutdown():
        time.sleep(1)
    

