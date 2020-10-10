#!/usr/bin/env python3

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
from math import sqrt


class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def add(self, v):
        return Vector2(self.x + v.x, self.y + v.y)

    def subtract(self, v):
        return Vector2(self.x - v.x, self.y - v.y)

    def dot(self, v):
        return self.x * v.x + self.y * v.y

    def norm(self):
        return sqrt(self.x * self.x + self.y * self.y)

    def norm_square(self):
        return self.x * self.x + self.y * self.y

    def print_point(self):
        print(str(self.x) + "\t" + str(self.y) + "\n")
