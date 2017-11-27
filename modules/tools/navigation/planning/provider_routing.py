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

import json
import threading
import math
from shapely.geometry import LineString, Point
from numpy.polynomial import polynomial as P

# sudo apt-get install libgeos-dev
# sudo pip install shapely

import numpy as np
from scipy.interpolate import UnivariateSpline, splev, splrep
from scipy.optimize import minimize


def error_function(c, x, y, t, k, w=None):
    """The error function to minimize"""
    diff = y - splev(x, (t, c, k))
    if w is None:
        diff = np.einsum('...i,...i', diff, diff)
    else:
        diff = np.dot(diff * diff, w)
    ddf = splev(x, (t, c, k), der=2)
    smoothness = 0
    for i in ddf:
        smoothness += i * i
    return np.abs(diff) + 1000 * smoothness


def optimized_spline(x, y, k=3, s=0, w=None):
    t, c0, k = splrep(x, y, w, k=k, s=s)
    x0 = x[0]
    constraint = {}
    constraint['type'] = 'eq'
    constraint['fun'] = lambda c: splev(x0, (t, c, k), der=1)
    constraints = [constraint]
    res = minimize(error_function, c0, (x, y, t, k, w), constraints=constraints)
    return UnivariateSpline._from_tck((t, res.x, k))


def euclidean(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


class RoutingProvider:
    def __init__(self):
        self.routing_str = None
        self.routing_points = []
        self.routing_lock = threading.Lock()

    def update(self, routing_str):
        self.routing_str = routing_str
        routing_json = json.loads(routing_str.data)
        routing_points = []
        for step in routing_json:
            points = step['polyline']['points']
            for point in points:
                routing_points.append(point)

        self.routing_lock.acquire()
        self.routing_points = routing_points
        self.routing_lock.release()

    def get_segment(self, utm_x, utm_y):
        if self.routing_str is None:
            return None
        point = Point(utm_x, utm_y)
        routing = LineString(self.routing_points)
        if routing.distance(point) > 10:
            return []
        if routing.length < 10:
            return []
        distance = routing.project(point)
        points = []
        total_length = routing.length
        for i in range(120):
            if (distance + i) >= total_length:
                break
            p = routing.interpolate(distance + i)
            points.append(p.coords[0])
        return points

    def get_local_segment(self, utm_x, utm_y, heading):
        points = self.get_segment(utm_x, utm_y)
        if points is None or len(points) < 30:
            return [], []
        points_x = []
        points_y = []
        for point in points:
            points_x.append(point[0])
            points_y.append(point[1])

        path_x = [x - utm_x for x in points_x]
        path_y = [y - utm_y for y in points_y]

        npath_x = []
        npath_y = []

        for i in range(len(path_x)):
            x = path_x[i]
            y = path_y[i]
            newx = x * math.cos(-heading) - y * math.sin(-heading)
            newy = y * math.cos(-heading) + x * math.sin(-heading)
            # newx = x * math.cos(- heading + 1.570796) - y * math.sin(
            #    -heading + 1.570796)
            # newy = y * math.cos(- heading + 1.570796) + x * math.sin(
            #    -heading + 1.570796)
            npath_x.append(newx)
            npath_y.append(newy)
        return npath_x, npath_y

    def get_local_segment_spline(self, utm_x, utm_y, heading):
        local_seg_x, local_seg_y = self.get_local_segment(utm_x, utm_y, heading)
        cut_idx = len(local_seg_x)
        for i in range(len(local_seg_x) - 1):
            if local_seg_x[i + 1] < local_seg_x[i]:
                cut_idx = i + 1
                break
        local_seg_x = local_seg_x[0:cut_idx]
        local_seg_y = local_seg_y[0:cut_idx]

        if len(local_seg_x) <= 10:
            return [], []
        k = 3
        n = len(local_seg_x)
        std = 0.5
        sp = optimized_spline(local_seg_x, local_seg_y, k, s=n * std)
        X = np.linspace(0, len(local_seg_x), len(local_seg_x))
        return X, sp(X)

    def get_smooth_local_segment(self, utm_x, utm_y, heading):
        local_seg_x, local_seg_y = self.get_local_segment(utm_x, utm_y, heading)
        if len(local_seg_x) <= 0:
            return None
        w = []
        point_num = len(local_seg_x)
        for i in range(point_num):
            w.append((point_num - i) ** 3)
        c, stats = P.polyfit(local_seg_x, local_seg_y, 3, full=True)
        return c
