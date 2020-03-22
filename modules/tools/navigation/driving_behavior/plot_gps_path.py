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
import pyproj
import matplotlib.pyplot as plt

projector = pyproj.Proj(proj='utm', zone=10, ellps='WGS84')
fig = plt.figure()
ax = plt.subplot2grid((1, 1), (0, 0))
styles = ['r-', 'b-']

i = 0
for fn in sys.argv[1:]:
    X = []
    Y = []
    f = open(fn, 'r')
    for line in f:
        line = line.replace('\n', '')
        vals = line.split(",")
        if len(vals) < 3:
            continue
        print(float(vals[-2]), float(vals[-1]))
        x, y = projector(float(vals[-1]), float(vals[-2]))
        print(x, y)
        X.append(x)
        Y.append(y)
    f.close()
    ax.plot(X, Y, styles[i % len(styles)], lw=3, alpha=0.8)
    i += 1
ax.axis('equal')
plt.show()
