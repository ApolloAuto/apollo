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

from modules.common_msgs.map_msgs import map_pb2
from modules.common_msgs.map_msgs import map_lane_pb2
from modules.common_msgs.map_msgs import map_road_pb2
from shapely.geometry import LineString, Point

LANE_WIDTH = 3.3


def convert(p, p2, distance):
    delta_y = p2.y - p.y
    delta_x = p2.x - p.x
    # print math.atan2(delta_y, delta_x)
    left_angle = math.atan2(delta_y, delta_x) + math.pi / 2.0
    right_angle = math.atan2(delta_y, delta_x) - math.pi / 2.0
    # print angle
    lp = []
    lp.append(p.x + (math.cos(left_angle) * distance))
    lp.append(p.y + (math.sin(left_angle) * distance))

    rp = []
    rp.append(p.x + (math.cos(right_angle) * distance))
    rp.append(p.y + (math.sin(right_angle) * distance))
    return lp, rp


def shift(p, p2, distance, isleft=True):
    delta_y = p2.y - p.y
    delta_x = p2.x - p.x
    # print math.atan2(delta_y, delta_x)
    angle = 0
    if isleft:
        angle = math.atan2(delta_y, delta_x) + math.pi / 2.0
    else:
        angle = math.atan2(delta_y, delta_x) - math.pi / 2.0
    # print angle
    p1n = []
    p1n.append(p.x + (math.cos(angle) * distance))
    p1n.append(p.y + (math.sin(angle) * distance))

    p2n = []
    p2n.append(p2.x + (math.cos(angle) * distance))
    p2n.append(p2.y + (math.sin(angle) * distance))
    return Point(p1n), Point(p2n)


def create_lane(map, id):
    lane = map.lane.add()
    lane.id.id = str(id)
    lane.type = map_lane_pb2.Lane.CITY_DRIVING
    lane.direction = map_lane_pb2.Lane.FORWARD
    lane.length = 100.0
    lane.speed_limit = 20.0
    lane.turn = map_lane_pb2.Lane.NO_TURN
    #lane.predecessor_id.add().id = str(id - 1)
    #lane.successor_id.add().id = str(id + 1)
    left_boundary = lane.left_boundary.curve.segment.add()
    right_boundary = lane.right_boundary.curve.segment.add()
    central = lane.central_curve.segment.add()
    central.length = 100.0

    type = lane.left_boundary.boundary_type.add()
    type.s = 0
    type.types.append(map_lane_pb2.LaneBoundaryType.DOTTED_YELLOW)
    lane.right_boundary.length = 100.0

    type = lane.right_boundary.boundary_type.add()
    type.s = 0
    type.types.append(map_lane_pb2.LaneBoundaryType.DOTTED_YELLOW)
    lane.left_boundary.length = 100.0

    return lane, central, left_boundary, right_boundary


fpath = sys.argv[1]
f = open(fpath, 'r')
points = []
for line in f:
    line = line.replace("\n", '')
    data = line.split(',')
    x = float(data[0])
    y = float(data[1])
    points.append((x, y))

path = LineString(points)
length = int(path.length)

extra_roi_extension = float(sys.argv[3])

fmap = open(sys.argv[2], 'w')
id = 0
map = map_pb2.Map()
road = map.road.add()
road.id.id = "1"
section = road.section.add()
section.id.id = "2"
lane = None
for i in range(length - 1):
    if i % 100 == 0:
        id += 1
        if lane is not None:
            lane.successor_id.add().id = str(id)
        lane, central, left_boundary, right_boundary = create_lane(map, id)
        section.lane_id.add().id = str(id)

        left_edge = section.boundary.outer_polygon.edge.add()
        left_edge.type = map_road_pb2.BoundaryEdge.LEFT_BOUNDARY
        left_edge_segment = left_edge.curve.segment.add()

        right_edge = section.boundary.outer_polygon.edge.add()
        right_edge.type = map_road_pb2.BoundaryEdge.RIGHT_BOUNDARY
        right_edge_segment = right_edge.curve.segment.add()

        if i > 0:
            lane.predecessor_id.add().id = str(id - 1)

            left_bound_point = left_boundary.line_segment.point.add()
            right_bound_point = right_boundary.line_segment.point.add()
            central_point = central.line_segment.point.add()

            right_edge_point = right_edge_segment.line_segment.point.add()
            left_edge_point = left_edge_segment.line_segment.point.add()

            p = path.interpolate(i - 1)
            p2 = path.interpolate(i - 1 + 0.5)
            distance = LANE_WIDTH / 2.0

            lp, rp = convert(p, p2, distance)
            left_bound_point.y = lp[1]
            left_bound_point.x = lp[0]
            right_bound_point.y = rp[1]
            right_bound_point.x = rp[0]

            lp, rp = convert(p, p2, distance + extra_roi_extension)
            left_edge_point.y = lp[1]
            left_edge_point.x = lp[0]
            right_edge_point.y = rp[1]
            right_edge_point.x = rp[0]

            central_point.x = p.x
            central_point.y = p.y

            left_sample = lane.left_sample.add()
            left_sample.s = 0
            left_sample.width = LANE_WIDTH / 2.0

            right_sample = lane.right_sample.add()
            right_sample.s = 0
            right_sample.width = LANE_WIDTH / 2.0

    left_bound_point = left_boundary.line_segment.point.add()
    right_bound_point = right_boundary.line_segment.point.add()
    central_point = central.line_segment.point.add()

    right_edge_point = right_edge_segment.line_segment.point.add()
    left_edge_point = left_edge_segment.line_segment.point.add()

    p = path.interpolate(i)
    p2 = path.interpolate(i + 0.5)
    distance = LANE_WIDTH / 2.0
    lp, rp = convert(p, p2, distance)

    central_point.x = p.x
    central_point.y = p.y
    left_bound_point.y = lp[1]
    left_bound_point.x = lp[0]
    right_bound_point.y = rp[1]
    right_bound_point.x = rp[0]

    left_edge_point.y = lp[1]
    left_edge_point.x = lp[0]
    right_edge_point.y = rp[1]
    right_edge_point.x = rp[0]

    left_sample = lane.left_sample.add()
    left_sample.s = i % 100 + 1
    left_sample.width = LANE_WIDTH / 2.0

    right_sample = lane.right_sample.add()
    right_sample.s = i % 100 + 1
    right_sample.width = LANE_WIDTH / 2.0

fmap.write(str(map))
fmap.close()
