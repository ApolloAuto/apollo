#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

import numpy as np


class Latency:
    def __init__(self):
        self.latency_list = []

    def put(self, adc_trajectory):
        self.latency_list.append(adc_trajectory.latency_stats.total_time_ms)

    def get(self):
        if len(self.latency_list) > 0:
            planning_latency = {
                "max": max(self.latency_list),
                "min": min(self.latency_list),
                "avg": np.average(self.latency_list)
            }
        else:
            planning_latency = {
                "max": 0.0,
                "min": 0.0,
                "avg": 0.0
            }
        return planning_latency
