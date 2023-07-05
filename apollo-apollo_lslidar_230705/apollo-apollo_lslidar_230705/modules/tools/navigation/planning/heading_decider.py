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
import numpy.polynomial.polynomial as poly


class HeadingDecider:
    def __init__(self):
        self.mobileye_pb = None

    def get_path(self, x, y, path_length):
        ind = int(math.floor((abs(x[0]) * 100.0) / 1) + 1)
        newx = [0]
        newy = [0]
        w = [1000]
        if len(y) - ind > 0:
            for i in range(len(y) - ind):
                newx.append(x[i + ind])
                newy.append(y[i + ind])
                w.append(w[-1] - 10)
        else:
            newx.append(x[-1])
            newy.append(y[-1])
            w.append(w[-1] - 10)
        coefs = poly.polyfit(newy, newx, 4, w=w)  # x = f(y)
        nx = poly.polyval(y, coefs)
        return nx, y
