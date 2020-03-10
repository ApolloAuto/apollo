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

import itertools
import sys

import matplotlib.pyplot as plt

import common.proto_utils as proto_utils
import debug_topo
from modules.routing.proto.routing_pb2 import RoutingResponse
from modules.routing.proto.topo_graph_pb2 import Graph


color_iter = itertools.cycle(
    ['navy', 'c', 'cornflowerblue', 'gold', 'darkorange'])
g_central_curve_dict = {}
g_center_point_dict = {}


def get_center_of_passage_region(region):
    """Get center of passage region center curve"""
    center_points = [g_center_point_dict[seg.id] for seg in region.segment]
    return center_points[len(center_points) // 2]


def plot_region(region, color):
    "Plot passage region"
    for seg in region.segment:
        center_pt = debug_topo.plot_central_curve_with_s_range(
            g_central_curve_dict[seg.id], seg.start_s, seg.end_s, color=color)
        debug_topo.draw_id(center_pt, seg.id, 'r')
        g_center_point_dict[seg.id] = center_pt
        print('Plot lane id: %s, start s: %f, end s: %f' % (seg.id, seg.start_s,
                                                            seg.end_s))


def plot_lane_change(lane_change, passage_regions):
    """Plot lane change information"""
    st_idx = lane_change.start_passage_region_index
    ed_idx = lane_change.end_passage_region_index
    from_pt = get_center_of_passage_region(passage_regions[st_idx])
    to_pt = get_center_of_passage_region(passage_regions[ed_idx])
    plt.gca().annotate(
        "",
        xy=(to_pt[0], to_pt[1]),
        xytext=(from_pt[0], from_pt[1]),
        arrowprops=dict(
            facecolor='blue', edgecolor='none', alpha=0.7, shrink=0.05))


def plot_road(road):
    """Plot road"""
    for region in road.passage_region:
        plot_region(region, 'green')
    for lane_change in road.lane_change_info:
        plot_lane_change(lane_change, road.passage_region)


def plot_junction(junction):
    """Plot junction"""
    plot_region(junction.passage_region, 'red')


def plot_result(routing_result, central_curve_dict):
    """Plot routing result"""
    plt.close()
    plt.figure()
    for way in routing_result.route:
        if way.HasField("road_info"):
            plot_road(way.road_info)
        else:
            plot_junction(way.junction_info)

    plt.gca().set_aspect(1)
    plt.title('Passage region')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()

    plt.draw()


def print_help():
    """Print help information.

    Print help information of usage.

    Args:

    """
    print('usage:')
    print('     python debug_topo.py file_path, then', end=' ')
    print_help_command()


def print_help_command():
    """Print command help information.

    Print help information of command.

    Args:

    """
    print('type in command: [q] [r]')
    print('         q               exit')
    print('         p               plot passage region')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print_help()
        sys.exit(0)
    print('Please wait for loading data...')

    topo_graph_file = sys.argv[1]
    graph = proto_utils.get_pb_from_bin_file(topo_graph_file, Graph())
    g_central_curve_dict = {nd.lane_id: nd.central_curve for nd in graph.node}

    plt.ion()
    while 1:
        print_help_command()
        print('cmd>', end=' ')
        instruction = raw_input()
        argv = instruction.strip(' ').split(' ')
        if len(argv) == 1:
            if argv[0] == 'q':
                sys.exit(0)
            elif argv[0] == 'p':
                routing_result_file = sys.argv[2]
                result = proto_utils.get_pb_from_bin_file(routing_result_file,
                                                          RoutingResponse())
                plot_result(result, g_central_curve_dict)
            else:
                print('[ERROR] wrong command')
            continue

        else:
            print('[ERROR] wrong arguments')
            continue
