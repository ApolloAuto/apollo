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
        self.routing = None
        self.routing_lock = threading.Lock()
        self.SMOOTH_FORWARD_DIST = 150
        self.SMOOTH_BACKWARD_DIST = 150
        self.human = False

    def update(self, routing_str):
        self.routing_str = routing_str
        routing_json = json.loads(routing_str.data)
        routing_points = []
        self.human = False
        for step in routing_json:
            if step.get('human'):
                self.human = True
            points = step['polyline']['points']
            for point in points:
                routing_points.append(point)

        self.routing_lock.acquire()
        self.routing_points = routing_points
        self.routing_lock.release()
        self.routing = LineString(self.routing_points)

    def get_segment(self, utm_x, utm_y):
        if self.routing_str is None:
            return None
        point = Point(utm_x, utm_y)
        if self.routing.distance(point) > 10:
            return []
        if self.routing.length < 10:
            return []
        vehicle_distance = self.routing.project(point)
        points = []
        total_length = self.routing.length
        for i in range(self.SMOOTH_BACKWARD_DIST):
            backward_dist = vehicle_distance - self.SMOOTH_BACKWARD_DIST + i
            if backward_dist < 0:
                continue
            p = self.routing.interpolate(backward_dist)
            points.append(p.coords[0])

        for i in range(self.SMOOTH_FORWARD_DIST):
            forward_dist = vehicle_distance + i
            if forward_dist >= total_length:
                break
            p = self.routing.interpolate(forward_dist)
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
            npath_x.append(newx)
            npath_y.append(newy)
        return npath_x, npath_y

    def to_monotonic_segment(self, seg_x, seg_y):
        left_cut_idx = 0
        right_cut_idx = len(seg_x)
        for i in range(len(seg_x) - 1):
            if seg_x[i + 1] < seg_x[i]:
                if seg_x[i] >= 0:
                    right_cut_idx = i + 1
                    break
                else:
                    left_cut_idx = i + 1
        mono_seg_x = seg_x[left_cut_idx:right_cut_idx]
        mono_seg_y = seg_y[left_cut_idx:right_cut_idx]
        return mono_seg_x, mono_seg_y

    def get_local_ref(self, local_seg_x, local_seg_y):
        ref_x = []
        ref_y = []
        points = []
        for i in range(len(local_seg_x)):
            x = local_seg_x[i]
            y = local_seg_y[i]
            points.append((x, y))
        line = LineString(points)
        dist = line.project(Point((0, 0)))
        for i in range(int(line.length - dist) + 1):
            p = line.interpolate(i + dist)
            ref_x.append(p.x)
            ref_y.append(p.y)
        return ref_x, ref_y

    def get_local_segment_spline(self, utm_x, utm_y, heading):
        local_seg_x, local_seg_y = self.get_local_segment(utm_x, utm_y, heading)
        if len(local_seg_x) <= 10:
            return [], []
        if self.human:
            return self.get_local_ref(local_seg_x, local_seg_y)
        mono_seg_x, mono_seg_y = self.to_monotonic_segment(
            local_seg_x, local_seg_y)

        if len(mono_seg_x) <= 10:
            return [], []
        k = 3
        n = len(mono_seg_x)
        std = 0.5
        sp = optimized_spline(mono_seg_x, mono_seg_y, k, s=n * std)
        X = np.linspace(0, int(local_seg_x[-1]), int(local_seg_x[-1]))
        return X, sp(X)

    def get_local_segment_spline_debug(self, utm_x, utm_y, heading, k=3,
                                       std=0.5):
        local_seg_x, local_seg_y = self.get_local_segment(utm_x, utm_y, heading)
        mono_seg_x, mono_seg_y = self.to_monotonic_segment(
            local_seg_x, local_seg_y)

        if len(mono_seg_x) <= 10:
            return [], []
        n = len(mono_seg_x)
        sp = optimized_spline(mono_seg_x, mono_seg_y, k, s=n * std)
        X = np.linspace(int(mono_seg_x[0]), int(mono_seg_x[-1]),
                        int(mono_seg_x[-1]))
        return X, sp(X)
