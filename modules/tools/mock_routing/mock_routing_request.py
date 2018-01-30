#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
Generate Planning Path
"""

import argparse
import atexit
import os
import sys
import time

import rospy
import scipy.signal as signal
from numpy import genfromtxt

from modules.routing.proto import routing_pb2

def main():
    """
    Main rosnode
    """
    rospy.init_node('mock_routing_requester', anonymous=True)
    sequence_num = 0

    routing_request = routing_pb2.RoutingRequest()
    routing_request.header.timestamp_sec = rospy.get_time()
    routing_request.header.module_name = 'routing_request'
    routing_request.header.sequence_num = sequence_num
    sequence_num = sequence_num + 1

    start_point = routing_request.start
    end_point = routing_request.end

    """
    start_point.id = '94_1_0'
    start_point.s = 53
    end_point.id = '94_1_0'
    end_point.s = 50
    """
    start_point.id = '1_-1'
    start_point.s = 1
    end_point.id = '1_-1'
    end_point.s = 80

    request_publisher = rospy.Publisher(
            '/apollo/routing_request', routing_pb2.RoutingRequest, queue_size=1)
    time.sleep(2.0)
    request_publisher.publish(routing_request)


if __name__ == '__main__':
    main()

