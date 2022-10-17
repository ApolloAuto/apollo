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
import os
import sys

import gflags
import matplotlib.pyplot as plt

import modules.tools.routing.debug_topo as debug_topo
import modules.routing.proto.topo_graph_pb2 as topo_graph_pb2
import modules.tools.routing.util as util

color_iter = itertools.cycle(
    ['navy', 'c', 'cornflowerblue', 'gold', 'darkorange'])


def read_route(route_file_name):
    """Read route result text file"""
    fin = open(route_file_name)
    route = []
    for line in fin:
        lane = {}
        items = line.strip().split(',')
        lane['id'] = items[0]
        lane['is virtual'] = int(items[1])
        lane['start s'] = float(items[2])
        lane['end s'] = float(items[3])
        route.append(lane)
    return route


def plot_route(lanes, central_curve_dict):
    """Plot route result"""
    plt.close()
    plt.figure()
    for lane in lanes:
        lane_id = lane['id']
        if lane['is virtual']:
            color = 'red'
        else:
            color = 'green'
        mid_pt = debug_topo.plot_central_curve_with_s_range(
            central_curve_dict[lane_id],
            lane['start s'],
            lane['end s'],
            color=color)
        debug_topo.draw_id(mid_pt, lane_id, 'y')
    plt.gca().set_aspect(1)
    plt.title('Routing result')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()

    plt.draw()


def print_help_command():
    """Print command help information.

    Print help information of command.

    Args:

    """
    print('type in command: [q] [r]')
    print('         q               exit')
    print('         r               plot route result')
    print('         r_map           plot route result with map')


if __name__ == '__main__':
    map_dir = util.get_map_dir(sys.argv)
    graph = util.get_topodata(map_dir)
    base_map = util.get_mapdata(map_dir)
    route = util.get_routingdata()

    central_curves = {}
    for nd in graph.node:
        central_curves[nd.lane_id] = nd.central_curve

    plt.ion()
    while 1:
        print_help_command()
        print('cmd>', end=' ')
        instruction = raw_input()
        argv = instruction.strip(' ').split(' ')
        if len(argv) == 1:
            if argv[0] == 'q':
                sys.exit(0)
            elif argv[0] == 'r':
                plot_route(route, central_curves)
            elif argv[0] == 'r_map':
                plot_route(route, central_curves)
                util.draw_map(plt.gca(), base_map)
            else:
                print('[ERROR] wrong command')
            continue

        else:
            print('[ERROR] wrong arguments')
            continue
