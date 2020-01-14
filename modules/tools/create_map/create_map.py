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
"""
Create base map from localization and mobileye lane detection
"""

import argparse
import csv
import math
import numpy as np
import os
import rospy
import sys

from modules.map.proto.map_pb2 import Map
from modules.map.proto.map_lane_pb2 import LaneBoundaryType, Lane
from modules.map.proto.map_road_pb2 import BoundaryEdge, Road

from modules.routing.proto.routing_pb2 import LaneWaypoint
from modules.routing.proto.poi_pb2 import POI, Landmark


class DataPoint:
    """
    class of data sample (localization and mobileye lane detection)
    """

    def __init__(self):
        self.pos_x = 0.0  # localization
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.theta = 0.0  # heading
        self.dist_left = 0.0  # distance to left lane marking
        # confidence of left lane marking (0/1: low confidence, -1/-2: high confidence)
        self.conf_left = 0
        self.dist_right = 0.0  # distance to right lane marking
        # confidence of right lane marking (0/1: low confidence, -1/-2: high confidence)
        self.conf_right = 0
        self.width = 0.0  # lane width
        self.ratio = 0.0  # relative position within a lane (dist_left / width)
        self.center_x = 0.0  # point on the center line of current lane
        self.center_y = 0.0


def distance(x1, y1, x2, y2):
    """
    l2 distance
    """

    return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))


def interpolate_width(data, default_width):
    """
    fill 'width' field of all data samples by interpolation
    """

    # Collect a set of consecutive entries with low confidence on left OR right lane detection
    intervals = []
    interval_begin = -1
    interval_end = -1
    for (index, entry) in enumerate(data):
        if entry.conf_left >= 0 or entry.conf_right >= 0:
            if interval_begin < 0:
                interval_begin = index
            interval_end = index
        else:
            if interval_begin >= 0:
                intervals.append((interval_begin, interval_end))
                interval_begin = -1
                interval_end = -1
            entry.width = entry.dist_left + entry.dist_right
    if interval_begin >= 0:
        intervals.append((interval_begin, interval_end))

    # Iterate through intervals to interpolate width
    for interval in intervals:
        for index in range(interval[0], interval[1] + 1):
            if interval[0] == 0 and interval[1] == len(data) - 1:
                data[index].width = default_width
            else:
                if interval[0] == 0:
                    data[index].width = data[interval[1] + 1].width
                elif interval[1] == len(data) - 1:
                    data[index].width = data[interval[0] - 1].width
                else:
                    alpha = float(
                        index - interval[0] + 1) / (interval[1] - interval[0] + 2)
                    data[index].width = (
                        1.0 - alpha) * data[interval[0] - 1].width + alpha * data[interval[1] + 1].width

    # Fill in dist_left/right and conf_left/right using interpolated width
    for (index, entry) in enumerate(data):
        if entry.conf_left >= 0 and entry.conf_right < 0:
            entry.dist_left = entry.width - entry.dist_right
            entry.conf_left = -1
        elif entry.conf_left < 0 and entry.conf_right >= 0:
            entry.dist_right = entry.width - entry.dist_left
            entry.conf_right = -1


def interpolate_ratio(data, default_ratio):
    """
    fill 'ratio' field of all data samples by interpolation
    """

    # Collect a set of consecutive entries with low confidence on left AND right lane detection
    intervals = []
    interval_begin = -1
    interval_end = -1
    for (index, entry) in enumerate(data):
        if entry.conf_left >= 0 and entry.conf_right >= 0:
            if interval_begin < 0:
                interval_begin = index
            interval_end = index
        else:
            if interval_begin >= 0:
                intervals.append((interval_begin, interval_end))
                interval_begin = -1
                interval_end = -1
            entry.ratio = float(entry.dist_left) / entry.width
    if interval_begin >= 0:
        intervals.append((interval_begin, interval_end))

    # Iterate through intervals to interpolate ratio
    for interval in intervals:
        for index in range(interval[0], interval[1] + 1):
            if interval[0] == 0 and interval[1] == len(data) - 1:
                data[index].ratio = default_ratio
            else:
                if interval[0] == 0:
                    data[index].ratio = data[interval[1] + 1].ratio
                elif interval[1] == len(data) - 1:
                    data[index].ratio = data[interval[0] - 1].ratio
                else:
                    alpha = float(
                        index - interval[0] + 1) / (interval[1] - interval[0] + 2)
                    data[index].ratio = (
                        1.0 - alpha) * data[interval[0] - 1].ratio + alpha * data[interval[1] + 1].ratio

    # Fill in dist_left/right and conf_left/right using interpolated ratio
    for (index, entry) in enumerate(data):
        if entry.conf_left >= 0 and entry.conf_right >= 0:
            entry.dist_left = entry.width * entry.ratio
            entry.dist_right = entry.width - entry.dist_left
            entry.conf_left = -1
            entry.conf_right = -1


def compute_center(data):
    """
    fill 'center_x' and 'center_y' fields of all data samples
    """

    for entry in data:
        pos_x = entry.pos_x
        pos_y = entry.pos_y
        pos_z = entry.pos_z
        theta = entry.theta
        dist_left = entry.dist_left
        dist_right = entry.dist_right

        theta_left = theta + np.pi / 2.0
        pos_l_x = pos_x + dist_left * np.cos(theta_left)
        pos_l_y = pos_y + dist_left * np.sin(theta_left)

        theta_right = theta - np.pi / 2.0
        pos_r_x = pos_x + dist_right * np.cos(theta_right)
        pos_r_y = pos_y + dist_right * np.sin(theta_right)

        entry.center_x = (pos_l_x + pos_r_x) / 2.0
        entry.center_y = (pos_l_y + pos_r_y) / 2.0


def sample_data(data, sample_distance):
    """
    sample 'data' at the interval of 'sample_distance'
    """

    result = []

    if len(data) > 0:
        last_x = data[0].center_x
        last_y = data[0].center_y
        result.append(data[0])

        for entry in data[1:]:
            if distance(last_x, last_y, entry.center_x, entry.center_y) > sample_distance:
                result.append(entry)
                last_x = entry.center_x
                last_y = entry.center_y

    return result


def extract_data(data, dim):
    """
    extract dimension 'dim' (center_x, center_y or width) of 'data' into a list
    """

    result = []
    for entry in data:
        if dim == 'center_x':
            result.append(entry.center_x)
        elif dim == 'center_y':
            result.append(entry.center_y)
        elif dim == 'width':
            result.append(entry.width)
    return result


def laplacian_operator(data):
    """
    apply laplacian operator on data
    """

    lap = []
    lap.append(0.0)
    for index in range(1, len(data) - 1):
        lap.append((data[index + 1] + data[index - 1]) / 2.0 - data[index])
    lap.append(0.0)
    return lap


def laplacian_smooth(data, alpha=0.5, iterations=3):
    """
    apply laplacian smoothing on data
    """

    for iteration in range(iterations):
        lap = laplacian_operator(data)
        for index in range(len(data)):
            data[index] += alpha * lap[index]


def update_data(data, dim, new_data):
    """
    copy new_data to dimension 'dim' of 'data'
    """

    for entry, new_entry in zip(data, new_data):
        if dim == 'center_x':
            entry.center_x = new_entry
        elif dim == 'center_y':
            entry.center_y = new_entry
        elif dim == 'width':
            entry.width = new_entry


def smooth_dimension(data, dim):
    """
    smooth dimension 'dim' of 'data'
    """

    extracted_data = extract_data(data, dim)
    if dim == 'width':
        laplacian_smooth(extracted_data, 1.0, 1000)
    else:
        laplacian_smooth(extracted_data, 1.0, 1000)
    update_data(data, dim, extracted_data)


def smooth_center_width(data):
    """
    smooth centers and widths of data
    """

    smooth_dimension(data, 'center_x')
    smooth_dimension(data, 'center_y')
    smooth_dimension(data, 'width')


def split_data(data, max_lane_length):
    """
    split data into multiple lists, each of which is not longer than 'max_lane_length'
    """

    result = []
    current = []
    total_length = 0.0

    if len(data) > 0:
        last_x = data[0].center_x
        last_y = data[0].center_y
        current.append(data[0])

        for entry in data[1:]:
            current.append(entry)

            d = distance(last_x, last_y, entry.center_x, entry.center_y)
            total_length += d

            if total_length > max_lane_length:
                result.append(current)

                current = []
                current.append(entry)
                total_length = 0.0

            last_x = entry.center_x
            last_y = entry.center_y

        if total_length > 0.0:
            result.append(current)

    return result


def create_lane(data, offset, lane_count, left_lanes, right_lanes):
    """
    create a lane using 'data' whose lateral index is 'offset'
    offset = 0: center lane; offset < 0: left lanes; offset > 0: right lanes
    lane_count: longitutional index of lane (used for naming)
    left_lanes, right_lanes: number of left/right lanes (used for boundary types)
    """

    total_length = 0.0
    total_left_length = 0.0
    total_right_length = 0.0

    lane = Lane()
    lane.id.id = "lane_" + str(lane_count) + "_" + str(offset)

    lane_central_curve_seg = lane.central_curve.segment.add()

    start_heading = data[0].theta

    lane_left_boundary_curve_seg = lane.left_boundary.curve.segment.add()
    lane_left_boundary_curve_seg.heading = float(start_heading)
    lane_left_boundary_curve_seg.s = 0.0

    lane_right_boundary_curve_seg = lane.right_boundary.curve.segment.add()
    lane_right_boundary_curve_seg.heading = float(start_heading)
    lane_right_boundary_curve_seg.s = 0.0

    last_l_x = 0.0
    last_l_y = 0.0

    last_c_x = 0.0
    last_c_y = 0.0

    last_r_x = 0.0
    last_r_y = 0.0

    for (index, entry) in enumerate(data):
        theta = entry.theta
        theta_left = theta + np.pi / 2.0
        theta_right = theta - np.pi / 2.0

        pos_c_x = entry.center_x
        pos_c_y = entry.center_y

        pos_l_x = pos_c_x + entry.width * (0.5 - offset) * np.cos(theta_left)
        pos_l_y = pos_c_y + entry.width * (0.5 - offset) * np.sin(theta_left)

        pos_r_x = pos_c_x + entry.width * (0.5 + offset) * np.cos(theta_right)
        pos_r_y = pos_c_y + entry.width * (0.5 + offset) * np.sin(theta_right)

        pos_c_x = (pos_l_x + pos_r_x) / 2.0
        pos_c_y = (pos_l_y + pos_r_y) / 2.0

        if index == 0:
            lane_central_curve_seg.start_position.x = pos_c_x
            lane_central_curve_seg.start_position.y = pos_c_y

            lane_left_boundary_curve_seg.start_position.x = pos_l_x
            lane_left_boundary_curve_seg.start_position.y = pos_l_y

            lane_right_boundary_curve_seg.start_position.x = pos_r_x
            lane_right_boundary_curve_seg.start_position.y = pos_r_y

        else:
            d = distance(last_c_x, last_c_y, pos_c_x, pos_c_y)
            total_length += d

            d_left = distance(last_l_x, last_l_y, pos_l_x, pos_l_y)
            total_left_length += d_left

            d_right = distance(last_r_x, last_r_y, pos_r_x, pos_r_y)
            total_right_length += d_right

        point = lane_central_curve_seg.line_segment.point.add()
        point.x = pos_c_x
        point.y = pos_c_y

        point = lane_left_boundary_curve_seg.line_segment.point.add()
        point.x = pos_l_x
        point.y = pos_l_y

        point = lane_right_boundary_curve_seg.line_segment.point.add()
        point.x = pos_r_x
        point.y = pos_r_y

        sample = lane.left_sample.add()
        sample.s = total_length
        sample.width = entry.width / 2.0

        sample = lane.right_sample.add()
        sample.s = total_length
        sample.width = entry.width / 2.0

        last_l_x = pos_l_x
        last_l_y = pos_l_y

        last_r_x = pos_r_x
        last_r_y = pos_r_y

        last_c_x = pos_c_x
        last_c_y = pos_c_y

    lane_central_curve_seg.length = total_length
    lane_left_boundary_curve_seg.length = total_left_length
    lane_right_boundary_curve_seg.length = total_right_length

    boundary_type = lane.left_boundary.boundary_type.add()
    boundary_type.s = 0.0
    if offset == -left_lanes:
        boundary_type.types.append(LaneBoundaryType.DOUBLE_YELLOW)
    else:
        boundary_type.types.append(LaneBoundaryType.DOTTED_WHITE)

    lane.left_boundary.length = total_left_length

    boundary_type = lane.right_boundary.boundary_type.add()
    boundary_type.s = 0.0
    if offset == right_lanes:
        boundary_type.types.append(LaneBoundaryType.CURB)
    else:
        boundary_type.types.append(LaneBoundaryType.DOTTED_WHITE)

    lane.right_boundary.length = total_right_length

    lane.length = total_length
    lane.speed_limit = 29.06
    lane.type = Lane.CITY_DRIVING
    lane.turn = Lane.NO_TURN

    return lane


def create_road(data, left_lanes, right_lanes):
    """
    create a road using 'data'
    left_lanes, right_lanes: number of left/right lanes
    """
    road = Road()
    road.id.id = "road"
    section = road.section.add()
    section.id.id = "section"

    left_edge = section.boundary.outer_polygon.edge.add()
    left_edge.type = BoundaryEdge.LEFT_BOUNDARY

    right_edge = section.boundary.outer_polygon.edge.add()
    right_edge.type = BoundaryEdge.RIGHT_BOUNDARY

    total_left_length = 0.0
    total_right_length = 0.0

    start_heading = data[0].theta

    left_edge_curve_seg = left_edge.curve.segment.add()
    left_edge_curve_seg.heading = float(start_heading)
    left_edge_curve_seg.s = 0.0

    right_edge_curve_seg = right_edge.curve.segment.add()
    right_edge_curve_seg.heading = float(start_heading)
    right_edge_curve_seg.s = 0.0

    last_l_x = 0.0
    last_l_y = 0.0

    last_r_x = 0.0
    last_r_y = 0.0

    for (index, entry) in enumerate(data):
        theta = entry.theta
        theta_left = theta + np.pi / 2.0
        theta_right = theta - np.pi / 2.0

        pos_l_x = entry.center_x + entry.width * \
            (0.5 + left_lanes) * np.cos(theta_left)
        pos_l_y = entry.center_y + entry.width * \
            (0.5 + left_lanes) * np.sin(theta_left)

        pos_r_x = entry.center_x + entry.width * \
            (0.5 + right_lanes) * np.cos(theta_right)
        pos_r_y = entry.center_y + entry.width * \
            (0.5 + right_lanes) * np.sin(theta_right)

        if index == 0:
            left_edge_curve_seg.start_position.x = pos_l_x
            left_edge_curve_seg.start_position.y = pos_l_y

            right_edge_curve_seg.start_position.x = pos_r_x
            right_edge_curve_seg.start_position.y = pos_r_y

        else:
            d_left = distance(last_l_x, last_l_y, pos_l_x, pos_l_y)
            total_left_length += d_left

            d_right = distance(last_r_x, last_r_y, pos_r_x, pos_r_y)
            total_right_length += d_right

        point = left_edge_curve_seg.line_segment.point.add()
        point.x = pos_l_x
        point.y = pos_l_y

        point = right_edge_curve_seg.line_segment.point.add()
        point.x = pos_r_x
        point.y = pos_r_y

        last_l_x = pos_l_x
        last_l_y = pos_l_y

        last_r_x = pos_r_x
        last_r_y = pos_r_y

    left_edge_curve_seg.length = total_left_length
    right_edge_curve_seg.length = total_right_length

    return road


def main():
    parser = argparse.ArgumentParser(
        description='Generate Base Map from Recorded Localization and Mobileye Lane Detection')
    parser.add_argument(
        '-i',
        '--input_file',
        help='Recorded localization and mobileye lane detection in CSV format',
        type=str,
        default='/tmp/lane.csv')
    parser.add_argument(
        '--debug',
        help='Print debugging info in /tmp',
        action='store_true')
    parser.add_argument(
        '-o',
        '--output_file',
        help='Output file name of generated base map',
        type=str,
        default='modules/map/data/gen/base_map.txt')
    parser.add_argument(
        '-e',
        '--end_waypoint_file',
        help='Output file name of default end waypoint',
        type=str,
        default='modules/map/data/gen/default_end_way_point.txt')
    parser.add_argument(
        '--default_width',
        help='Default lane width in meters (only effective when mobileye lane detection fails for ALL frames)',
        type=float,
        default=3.5)
    parser.add_argument(
        '--sample_distance',
        help='minimum distance (in meters) of two adjacent samples of a lane',
        type=float,
        default=0.2)
    parser.add_argument(
        '--max_lane_length',
        help='maximum length (in meters) of a lane (longer lanes will be split)',
        type=float,
        default=100.0)
    parser.add_argument(
        '--left_lanes',
        help='Number of lanes on the left',
        type=int,
        default=0)
    parser.add_argument(
        '--right_lanes',
        help='Number of lanes on the right',
        type=int,
        default=0)
    args = vars(parser.parse_args())

    csv_file_name = args['input_file']
    map_file_name = args['output_file']
    waypoint_file_name = args['end_waypoint_file']
    default_width = args['default_width']
    debug_option = args['debug']
    sample_distance = args['sample_distance']
    max_lane_length = args['max_lane_length']
    left_lanes = args['left_lanes']
    right_lanes = args['right_lanes']

    default_ratio = 0.5
    temp_csv_file_name = '/tmp/lane_interpolation.csv'

    rows = []
    with open(csv_file_name, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            rows.append(row)

    # Extract data samples
    data = []
    for row in rows[1:]:
        entry = DataPoint()
        entry.pos_x = float(row[0])
        entry.pos_y = float(row[1])
        entry.pos_z = float(row[2])
        entry.theta = float(row[3])
        entry.dist_left = abs(float(row[4]))
        entry.conf_left = int(row[5])
        if entry.dist_left < 0.1:
            entry.conf_left = 0
        entry.dist_right = abs(float(row[6]))
        entry.conf_right = int(row[7])
        if entry.dist_right < 0.1:
            entry.conf_right = 0
        entry.width = default_width
        entry.ratio = default_ratio
        data.append(entry)

    # Fill in widths using interpolation
    interpolate_width(data, default_width)
    # Fill in ratios using interpolation
    interpolate_ratio(data, default_ratio)
    # Fill in centers
    compute_center(data)

    # Sample data at the interval of sample_distance
    data = sample_data(data, sample_distance)
    # Smooth center curves and widths
    smooth_center_width(data)

    # Output debug info if necessary
    if debug_option:
        with open(temp_csv_file_name, 'w') as csvfile:
            for row in data:
                csvfile.write(
                    "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" %
                    (row.pos_x, row.pos_y, row.pos_z, row.theta, row.dist_left, row.conf_left, row.dist_right, row.conf_right, row.width, row.ratio, row.center_x, row.center_y))

    # Split data samples into lists with maximum length of max_lane_length
    list_data = split_data(data, max_lane_length)

    # Create individual lanes
    lane_sets = []
    for (lane_count, lane_data) in enumerate(list_data):
        lane_set = []
        for offset in range(-left_lanes, right_lanes + 1):
            lane_set.append(create_lane(lane_data, offset,
                                        lane_count, left_lanes, right_lanes))
        lane_sets.append(lane_set)

    # Create road
    road = create_road(data, left_lanes, right_lanes)

    # Create map
    mp = Map()
    mp.header.version = "1.400000"
    mp.header.date = "20170919"
    mp.header.district = "101"

    # Set up predecessors, successors, left/right neighbors
    for lane_count in range(len(lane_sets)):
        for lane_offset in range(len(lane_sets[lane_count])):
            if lane_count != 0:
                lane_sets[lane_count][lane_offset].predecessor_id.add(
                ).id = lane_sets[lane_count - 1][lane_offset].id.id
            if lane_count != len(lane_sets) - 1:
                lane_sets[lane_count][lane_offset].successor_id.add(
                ).id = lane_sets[lane_count + 1][lane_offset].id.id
            if lane_offset != 0:
                lane_sets[lane_count][lane_offset].left_neighbor_forward_lane_id.add(
                ).id = lane_sets[lane_count][lane_offset - 1].id.id
            if lane_offset != len(lane_sets[lane_count]) - 1:
                lane_sets[lane_count][lane_offset].right_neighbor_forward_lane_id.add(
                ).id = lane_sets[lane_count][lane_offset + 1].id.id

    # Add road/lanes to map and let road contain lanes
    mp.road.extend([road])
    for lane_set in lane_sets:
        for lane in lane_set:
            mp.road[0].section[0].lane_id.add().id = lane.id.id
            mp.lane.extend([lane])

    # Output map
    with open(map_file_name, "w") as f:
        f.write(mp.__str__())

    # Create default end_way_point using the farthest point of last central lane
    last_central_lane = lane_sets[-1][left_lanes]

    poi = POI()
    landmark = poi.landmark.add()
    landmark.name = "default"
    waypoint = landmark.waypoint.add()
    waypoint.id = last_central_lane.id.id
    waypoint.s = last_central_lane.length
    waypoint.pose.x = last_central_lane.central_curve.segment[0].line_segment.point[-1].x
    waypoint.pose.y = last_central_lane.central_curve.segment[0].line_segment.point[-1].y

    # Output default end_way_point
    with open(waypoint_file_name, "w") as f:
        f.write(poi.__str__())


if __name__ == '__main__':
    main()
