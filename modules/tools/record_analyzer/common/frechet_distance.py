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

import math

import numpy as np


def euclidean_distance(pt1, pt2):
    return math.sqrt((pt2[0] - pt1[0]) * (pt2[0]-pt1[0]) +
                     (pt2[1] - pt1[1]) * (pt2[1] - pt1[1]))


def _c(ca, i, j, P, Q):
    if ca[i, j] > -1:
        return ca[i, j]
    elif i == 0 and j == 0:
        ca[i, j] = euclidean_distance(P[0], Q[0])
    elif i > 0 and j == 0:
        ca[i, j] = max(_c(ca, i-1, 0, P, Q), euclidean_distance(P[i], Q[0]))
    elif i == 0 and j > 0:
        ca[i, j] = max(_c(ca, 0, j-1, P, Q), euclidean_distance(P[0], Q[j]))
    elif i > 0 and j > 0:
        ca[i, j] = max(min(_c(ca, i-1, j, P, Q),
                           _c(ca, i-1, j-1, P, Q),
                           _c(ca, i, j-1, P, Q)),
                       euclidean_distance(P[i], Q[j]))
    else:
        ca[i, j] = float("inf")
    return ca[i, j]


def frechet_distance(P, Q):
    ca = np.ones((len(P), len(Q)))
    ca = np.multiply(ca, -1)
    dist = None
    try:
        dist = _c(ca, len(P)-1, len(Q)-1, P, Q)
    except:
        print("calculate frechet_distance exception.")
    return dist


if __name__ == "__main__":
    """test"""
    P = [[1, 1], [2, 1], [2, 2]]
    Q = [[2, 2], [0, 1], [2, 4]]
    print(frechet_distance(P, Q))  # 2

    P = [[1, 1], [2, 1], [2, 2]]
    Q = [[1, 1], [2, 1], [2, 2]]
    print(frechet_distance(P, Q))  # 0
