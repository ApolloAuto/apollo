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
    print "usage: python plot_results.py result.csv"
    sys.exit()

fn = sys.argv[1]

speed_table = {}
f = open(fn, 'r')
for line in f:
    items = line.split(',')
    cmd = round(float(items[0]))
    speed = float(items[1])
    acc = round(float(items[2]), 2)
    if speed in speed_table:
        cmd_table = speed_table[speed]
        if cmd in cmd_table:
            cmd_table[cmd].append(acc)
        else:
            cmd_table[cmd] = [acc]
    else:
        cmd_table = {}
        cmd_table[cmd] = [acc]
        speed_table[speed] = cmd_table
f.close()

for speed, cmd_dict in speed_table.items():
    speed_list = []
    acc_list = []
    for cmd, accs in cmd_dict.items():
        for acc in accs:
            speed_list.append(speed)
            acc_list.append(acc)
    plt.plot(speed_list, acc_list, 'b.')
plt.show()
