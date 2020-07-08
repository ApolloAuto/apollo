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
"""Show road."""

import sys

import matplotlib.pyplot as plt

import modules.tools.common.proto_utils as proto_utils
import modules.tools.routing.util as util


g_color = [
    'navy', 'c', 'cornflowerblue', 'gold', 'darkorange', 'darkviolet',
    'aquamarine', 'firebrick', 'limegreen'
]


def draw_line(line_segment, color):
    """
    :param line_segment:
    :return: none
    """
    px, py = proto_utils.flatten(line_segment.point, ['x', 'y'])
    px, py = downsample_array(px), downsample_array(py)
    plt.gca().plot(px, py, lw=10, alpha=0.8, color=color)
    return px[len(px) // 2], py[len(py) // 2]


def draw_arc(arc):
    """
    :param arc: proto obj
    :return: none
    """
    xy = (arc.center.x, arc.center.y)
    start = 0
    end = 0
    if arc.start_angle < arc.end_angle:
        start = arc.start_angle / math.pi * 180
        end = arc.end_angle / math.pi * 180
    else:
        end = arc.start_angle / math.pi * 180
        start = arc.end_angle / math.pi * 180

    pac = mpatches.Arc(
        xy, arc.radius * 2, arc.radius * 2, angle=0, theta1=start, theta2=end)

    plt.gca().add_patch(pac)


def downsample_array(array):
    """down sample given array"""
    skip = 5
    result = array[::skip]
    result.append(array[-1])
    return result


def draw_boundary(line_segment):
    """
    :param line_segment:
    :return:
    """
    px, py = proto_utils.flatten(line_segment.point, ['x', 'y'])
    px, py = downsample_array(px), downsample_array(py)
    plt.gca().plot(px, py, 'k')


def draw_id(x, y, id_string):
    """Draw id_string on (x, y)"""
    plt.annotate(
        id_string,
        xy=(x, y),
        xytext=(40, -40),
        textcoords='offset points',
        ha='right',
        va='bottom',
        bbox=dict(boxstyle='round,pad=0.5', fc='green', alpha=0.5),
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))


def get_road_index_of_lane(lane_id, road_lane_set):
    """Get road index of lane"""
    for i, lane_set in enumerate(road_lane_set):
        if lane_id in lane_set:
            return i
    return -1


def draw_map(drivemap):
    """ draw map from mapfile"""
    print('Map info:')
    print('\tVersion:\t', end=' ')
    print(drivemap.header.version)
    print('\tDate:\t', end=' ')
    print(drivemap.header.date)
    print('\tDistrict:\t', end=' ')
    print(drivemap.header.district)

    road_lane_set = []
    for road in drivemap.road:
        lanes = []
        for sec in road.section:
            lanes.extend(proto_utils.flatten(sec.lane_id, 'id'))
        road_lane_set.append(lanes)

    for lane in drivemap.lane:
        for curve in lane.central_curve.segment:
            if curve.HasField('line_segment'):
                road_idx = get_road_index_of_lane(lane.id.id, road_lane_set)
                if road_idx == -1:
                    print('Failed to get road index of lane')
                    sys.exit(-1)
                center_x, center_y = draw_line(curve.line_segment,
                                               g_color[road_idx % len(g_color)])
                draw_id(center_x, center_y, str(road_idx))
                # break
            # if curve.HasField('arc'):
            #    draw_arc(curve.arc)

        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                draw_boundary(curve.line_segment)

        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                draw_boundary(curve.line_segment)
                # break

    return drivemap


if __name__ == "__main__":
    print("Reading map data")
    map_dir = util.get_map_dir(sys.argv)
    base_map = util.get_mapdata(map_dir)
    print("Done reading map data")
    plt.subplots()
    draw_map(base_map)
    plt.axis('equal')
    plt.show()
