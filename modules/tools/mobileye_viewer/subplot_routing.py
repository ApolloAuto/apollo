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
import math


class SubplotRouting:
    def __init__(self, ax):
        self.ax = ax
        self.routing_dot, = ax.plot([0], [0], 'ro', lw=3, alpha=0.4)
        self.routing_line, = ax.plot([0], [0], 'b-', lw=1, alpha=1)
        self.segment_line, = ax.plot([0], [0], 'bo', lw=1, alpha=1)

    def show(self, routing_data):
        routing_data.routing_data_lock.acquire()
        self.routing_dot.set_xdata(routing_data.routing_x)
        self.routing_dot.set_ydata(routing_data.routing_y)
        self.routing_line.set_xdata(routing_data.routing_x)
        self.routing_line.set_ydata(routing_data.routing_y)
        self.segment_line.set_xdata(routing_data.segment_x)
        self.segment_line.set_ydata(routing_data.segment_y)

        routing_data.routing_data_lock.release()
        self.ax.autoscale_view()
        self.ax.relim()
