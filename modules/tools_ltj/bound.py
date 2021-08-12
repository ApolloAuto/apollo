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

import sys
import matplotlib.pyplot as plt

f = open("/apollo/data/bound", 'r')
xs = []
ys = []
fig = plt.figure()
ax = plt.subplot2grid((1, 1), (0, 0))
for line in f:
    if line.find("--")!=-1:
        print("dd ",line)
        ax.axis('equal')
        ax.plot(xs, ys, "-", lw=1)
        xs=[]
        ys=[]
        continue
    line = line.replace("\n", '')
    data = line.split(',')
    x = float(data[0])
    y = float(data[1])
    print("xy:",x,y)
    xs.append(x)
    ys.append(y)
f.close()





while(1):
    pos=plt.ginput(1)
    if len(pos)==0:
        continue
    plt.plot(pos[0][0],pos[0][1],"om")
    print("%10.3f,%10.3f"%(pos[0][0],pos[0][1]))
plt.show()

