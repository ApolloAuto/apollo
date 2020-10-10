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

import sys
from shapely.geometry import LineString, Point
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fpath = sys.argv[1]
    f = open(fpath, 'r')
    points_x = []
    points_y = []
    points = []

    for line in f:
        line = line.replace("\n", '')
        if len(line.strip()) == 0:
            continue
        data = line.split(',')
        x = float(data[0])
        y = float(data[1])
        points_x.append(x)
        points_y.append(y)
        points.append((x, y))
    f.close()
    line_string = LineString(points)
    new_px = []
    new_py = []
    f = open("processed_" + fpath.split("/")[-1], 'w')
    for i in range(int(line_string.length)):
        p = line_string.interpolate(i)
        new_px.append(p.x)
        new_py.append(p.y)
        f.write(str(p.x) + "," + str(p.y) + "\n")
    f.close()
    print(len(points_x))
    print(len(new_px))
    plt.figure()
    plt.plot(points_x, points_y, '-r', lw=1, label='raw')
    plt.plot(new_px, new_py, '-g', label='processed')
    plt.legend(loc='best')
    plt.show()
