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
import json
import matplotlib.pyplot as plt

import numpy
from scipy.signal import butter, lfilter, freqz


def get_s_xy_kappa(fn, ax, ax2):
    f = open(fn, 'r')
    xs = []
    ys = []
    ks = []
    theta = []
    s = []
    cnt = 0
    for line in f:
        cnt += 1
        if cnt < 3:
            continue
        line = line.replace("\n", '')
        data = json.loads(line)
        ks.append(data['kappa'])
        s.append(data['s'])
        xs.append(data['x'])
        ys.append(data['y'])
        theta.append(data['theta'])
    f.close()
    return s, xs, ys, theta, ks


def plot_raw_path(fn, ax):
    f = open(fn, 'r')
    xs = []
    ys = []
    for line in f:
        line = line.replace("\n", '')
        data = line.split(',')
        x = float(data[0])
        y = float(data[1])
        xs.append(x)
        ys.append(y)
    f.close()
    ax.plot(xs, ys, "r-", lw=3, alpha=0.8)


if __name__ == "__main__":
    fig = plt.figure()
    ax = plt.subplot2grid((2, 2), (0, 0))
    ax2 = plt.subplot2grid((2, 2), (0, 1))
    ax3 = plt.subplot2grid((2, 2), (1, 0))
    ax4 = plt.subplot2grid((2, 2), (1, 1))

    styles = ["bo", "ro", "yo"]
    i = 0

    fn = sys.argv[1]
    plot_raw_path(fn, ax)

    fn = sys.argv[2]
    s, xs, ys, theta, ks = get_s_xy_kappa(fn, ax, ax2)
    ax.plot(xs, ys, "b-", lw=8, alpha=0.5)
    ax.set_title("x-y")

    ax2.plot(s, ks, 'k-')
    ax2.set_title("s-kappa")
    ax2.axhline(y=0.0, color='b', linestyle='-')

    ax3.plot(s, theta, 'k-')
    ax3.set_title("s-theta")

    ax4.plot(s, s, 'k-')
    ax4.set_title("s-s")

    if len(sys.argv) >= 4:
        fn = sys.argv[3]
        s, xs, ys, theta, ks = get_s_xy_kappa(fn, ax, ax2)
        ax.plot(xs, ys, "r-", lw=8, alpha=0.5)
        ax.set_title("x-y")

        ax2.plot(s, ks, 'r-')
        ax2.set_title("s-kappa")
        ax2.axhline(y=0.0, color='b', linestyle='-')

        ax3.plot(s, theta, 'r-')
        ax3.set_title("s-theta")

        ax4.plot(s, s, 'r-')
        ax4.set_title("s-s")

    ax.axis('equal')
    plt.show()
