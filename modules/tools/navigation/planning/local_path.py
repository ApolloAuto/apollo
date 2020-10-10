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


class LocalPath:
    def __init__(self, points):
        self.points = points

    def init_y(self):
        if len(self.points) > 0:
            return self.points[0][1]
        return None

    def get_xy(self):
        x = []
        y = []
        for p in self.points:
            x.append(p[0])
            y.append(p[1])
        return x, y

    def range(self):
        return len(self.points) - 1

    def shift(self, dist):
        for i in range(len(self.points)):
            self.points[i][1] += dist

    def cut(self, dist):
        pass

    def resample(self):
        pass

    def merge(self, local_path, weight):
        for i in range(len(self.points)):
            y = self.points[i][1]
            if i < len(local_path.points):
                y2 = local_path.points[i][1] * weight
                self.points[i][1] = (y + y2) / (1 + weight)
