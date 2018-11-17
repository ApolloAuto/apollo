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

import threading
import json


class RoutingData:
    def __init__(self, routing_str=None):
        self.routing_str = routing_str
        self.routing_debug_str = None
        self.routing_data_lock = threading.Lock()
        self.routing_x = []
        self.routing_y = []
        self.segment_x = []
        self.segment_y = []

    def update_navigation(self, navigation_info_pb):
        routing_x = []
        routing_y = []
        for navi_path in navigation_info_pb.navigation_path:
            for path_point in navi_path.path.path_point:
                routing_x.append(path_point.x)
                routing_y.append(path_point.y)
        self.routing_data_lock.acquire()
        self.routing_x = routing_x
        self.routing_y = routing_y
        self.routing_data_lock.release()

    def update(self, routing_str):
        self.routing_str = routing_str
        routing_json = json.loads(routing_str.data)
        routing_x = []
        routing_y = []
        for step in routing_json:
            points = step['polyline']['points']
            for point in points:
                routing_x.append(point[0])
                routing_y.append(point[1])

        self.routing_data_lock.acquire()
        self.routing_x = routing_x
        self.routing_y = routing_y
        self.routing_data_lock.release()

    def update_debug(self, routing_debug_str):
        self.routing_debug_str = routing_debug_str
        segment_json = json.loads(routing_debug_str.data)
        if segment_json is None:
            return
        segment_x = []
        segment_y = []
        for point in segment_json:
            segment_x.append(point[0])
            segment_y.append(point[1])

        self.routing_data_lock.acquire()
        self.segment_x = segment_x
        self.segment_y = segment_y
        self.routing_data_lock.release()
