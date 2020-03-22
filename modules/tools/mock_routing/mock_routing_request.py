#!/usr/bin/env python3

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
Insert routing request
Usage:
    mock_routing_request.py
"""
import argparse
import os
import sys
import time

from cyber_py3 import cyber
from cyber_py3 import cyber_time
from modules.routing.proto import routing_pb2


def main():
    """
    Main rosnode
    """
    cyber.init()
    node = cyber.Node("mock_routing_requester")
    sequence_num = 0

    routing_request = routing_pb2.RoutingRequest()

    routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    routing_request.header.module_name = 'routing_request'
    routing_request.header.sequence_num = sequence_num
    sequence_num = sequence_num + 1

    waypoint = routing_request.waypoint.add()
    waypoint.pose.x = 587696.82286
    waypoint.pose.y = 4141446.66696
    waypoint.id = '1-1'
    waypoint.s = 1

    waypoint = routing_request.waypoint.add()
    waypoint.pose.x = 586948.740120
    waypoint.pose.y = 4141171.118641
    waypoint.id = '1-1'
    waypoint.s = 80

    writer = node.create_writer('/apollo/routing_request',
                                routing_pb2.RoutingRequest)
    time.sleep(2.0)
    print("routing_request", routing_request)
    writer.write(routing_request)


if __name__ == '__main__':
    main()
