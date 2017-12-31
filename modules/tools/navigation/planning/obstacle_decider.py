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

from shapely.geometry import LineString
from shapely.geometry import Point


class ObstacleDecider:
    def __init__(self):
        self.obstacle_lat_ttc = {}
        self.obstacle_lon_ttc = {}
        self.obstacle_lat_dist = {}
        self.obstacle_lon_dist = {}
        self.front_edge_to_center = 3.89
        self.back_edge_to_center = 1.043
        self.left_edge_to_center = 1.055
        self.right_edge_to_center = 1.055
        self.LAT_DIST = 1.0
        self.mobileye = None
        self.path_obstacle_processed = False

    def update(self, mobileye):
        self.mobileye = mobileye
        self.path_obstacle_processed = False

    def process_path_obstacle(self, path_x, path_y):
        if self.path_obstacle_processed:
            return

        self.obstacle_lat_dist = {}

        path = []
        self.mobileye.process_obstacles()
        for i in range(len(path_x)):
            path.append((path_x[i], path_y[i]))
        line = LineString(path)

        for obstacle in self.mobileye.obstacles:
            point = Point(obstacle.x, obstacle.y)
            dist = line.distance(point)
            if dist < self.left_edge_to_center + self.LAT_DIST:
                proj_len = line.project(point)
                if proj_len == 0 or proj_len >= line.length:
                    continue
                p1 = line.interpolate(proj_len)
                if (proj_len + 1) > line.length:
                    p2 = line.interpolate(line.length)
                else:
                    p2 = line.interpolate(proj_len + 1)
                d = (point.x - p1.x) * (p2.y - p1.y) - (point.y - p1.y) * (
                    p2.x - p1.x)
                if d < 0:
                    dist *= -1
                self.obstacle_lat_dist[obstacle.obstacle_id] = dist

        self.path_obstacle_processed = True
