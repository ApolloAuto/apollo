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
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm as cmx
from matplotlib import colors as mcolors

markers = [
    "o", "v", "^", "<", ">", "1", "2", "3", "4", "8", "s", "p", "*", "+", "x",
    "d", "|", "_"
]

if len(sys.argv) < 2:
    print("Usage: python plot_results.py result.csv")
    sys.exit(0)

with open(sys.argv[1], 'r') as f:
    cmd_table = {}

    for line in f:
        items = line.split(',')
        cmd = round(float(items[0]))
        speed = float(items[1])
        acc = float(items[2])
        if cmd in cmd_table:
            speed_table = cmd_table[cmd]
            if speed in speed_table:
                speed_table[speed].append(acc)
            else:
                speed_table[speed] = [acc]
        else:
            speed_table = {}
            speed_table[speed] = [acc]
            cmd_table[cmd] = speed_table

NCURVES = len(cmd_table)
np.random.seed(101)
curves = [np.random.random(20) for i in range(NCURVES)]
values = list(range(NCURVES))
jet = cm = plt.get_cmap('brg')
cNorm = mcolors.Normalize(vmin=0, vmax=values[-1])
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)

cnt = 0
cmds = list(cmd_table.keys())
cmds.sort()

fig, ax = plt.subplots()
for cmd in cmds:
    print('ctrl cmd = %s' % cmd)
    speed_table = cmd_table[cmd]
    X = []
    Y = []
    speeds = list(speed_table.keys())
    speeds.sort()
    for speed in speeds:
        X.append(speed)
        Y.append(np.mean(speed_table[speed]))
    colorVal = scalarMap.to_rgba(values[cnt])
    ax.plot(
        X,
        Y,
        c=colorVal,
        linestyle=':',
        marker=markers[cnt % len(markers)],
        label="cmd=" + str(cmd))
    cnt += 1

ax.legend(loc='upper center', shadow=True, bbox_to_anchor=(0.5, 1.1), ncol=5)

plt.ylabel("acc")
plt.xlabel("speed")
plt.grid()
plt.show()
