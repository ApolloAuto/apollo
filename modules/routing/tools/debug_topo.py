################################################################################
#
# Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
#
################################################################################
"""
This module provides routing topology file parse and plot functions.

Authors: fuxiaoxin(fuxiaoxin@baidu.com)
"""

import sys
import math
import itertools
import gen.topo_graph_pb2 as topo_graph_pb2
import matplotlib.pyplot as plt

color_iter = itertools.cycle(['navy', 'c', 'cornflowerblue', 'gold',
                              'darkorange'])

def downsample_array(array):
    """down sample given array"""
    skip = 5
    result = array[::skip]
    result.append(array[-1])
    return result


def calculate_s(px, py):
    """Calculate s array based on x and y arrays"""
    dis = 0.0
    ps = [dis]
    for i in range(len(px) - 1):
        gap = math.sqrt(pow(px[i + 1] - px[i], 2) + pow(py[i + 1] - py[i], 2))
        dis = dis + gap
        ps.append(dis)
    return ps


def extract_line(line):
    """extract line, return x array and y array"""
    px = []
    py = []
    for pt in line.point:
        px.append(float(pt.x))
        py.append(float(pt.y))
    return px, py


def draw_line(line, color):
    """draw line, return x array and y array"""
    px = []
    py = []
    for pt in line.point:
        px.append(float(pt.x))
        py.append(float(pt.y))
    px = downsample_array(px)
    py = downsample_array(py)
    plt.gca().plot(px, py, color=color, lw = 3, alpha=0.8)
    return px, py


def draw_arc(arc):
    """draw arc"""
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
        xy,
        arc.radius * 2,
        arc.radius * 2,
        angle = 0,
        theta1 = start,
        theta2 = end
    )

    plt.gca().add_patch(pac)


def draw_id(coordinate, id_string, color):
    """draw id"""
    x = coordinate[0]
    y = coordinate[1]
    plt.annotate(
        id_string,
        xy = (x, y), xytext = (30, 30),
        textcoords = 'offset points',
        bbox = dict(boxstyle = 'round,pad=0.5', fc = color, alpha = 0.5),
        arrowprops = dict(arrowstyle = '->', connectionstyle = 'arc3,rad=0'),
        horizontalalignment='right', verticalalignment='bottom')


def plot_central_curve_with_s_range(central_curve, start_s, end_s, color):
    """plot topology graph node with given start and end s, return middle point"""
    node_x = []
    node_y = []
    plot_length = 0.0
    for curve in central_curve.segment:
        px, py = extract_line(curve.line_segment)
        node_x = node_x + px
        node_y = node_y + py
    start_plot_index = 0
    end_plot_index = len(node_x)
    node_s = calculate_s(node_x, node_y)
    for i in range(len(node_s)):
        if node_s[i] >= start_s:
            start_plot_index = i
            break
    for i in range(len(node_s) - 1, -1, -1):
        if node_s[i] <= end_s:
            end_plot_index = i + 1
            break
    plt.gca().plot(node_x[start_plot_index:end_plot_index],
                   node_y[start_plot_index:end_plot_index],
                   color=color,
                   lw = 3,
                   alpha=0.8)
    mid_index = (start_plot_index + end_plot_index) // 2
    return [node_x[mid_index], node_y[mid_index]]


def plot_central_curve(central_curve, color):
    """plot topology graph node, return node middle point"""
    node_x = []
    node_y = []
    for curve in central_curve.segment:
        if curve.HasField('line_segment'):
            px, py = draw_line(curve.line_segment, color)
            node_x = node_x + px
            node_y = node_y + py
        if curve.HasField('arc'):
            draw_arc(curve.arc)
    return [node_x[len(node_x) / 2], node_y[len(node_y) / 2]]


def plot_node(node, color):
    """plot topology graph node"""
    mid_pt =  plot_central_curve(node.central_curve, color)
    draw_id(mid_pt, node.lane_id, 'green')
    return mid_pt


def plot_edge(edge, midddle_point_map):
    """plot topology graph edge"""
    if edge.direction_type == topo_graph_pb2.Edge.FORWARD:
        return
    # if lane change is allowed, draw an arrow from lane with from_id to lane with to_id
    from_id = edge.from_lane_id
    from_pt = midddle_point_map[from_id]
    to_id = edge.to_lane_id
    to_pt = midddle_point_map[to_id]
    plt.gca().annotate("",
                       xy=(to_pt[0], to_pt[1]),
                       xytext=(from_pt[0], from_pt[1]),
                       arrowprops=dict(arrowstyle="->",
                                       connectionstyle="arc3"))


def plot_all(graph):
    """plot topology graph"""
    plt.close()
    plt.figure()
    lane_middle_point_map = {}
    for i, (nd, color) in enumerate(zip(graph.node, color_iter)):
        nd_mid_pt = plot_node(nd, color)
        lane_middle_point_map[nd.lane_id] = nd_mid_pt
    for i, eg in enumerate(graph.edge):
        plot_edge(eg, lane_middle_point_map)
    plt.gca().set_aspect(1)
    plt.title('Routing topology graph')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()

    plt.draw()


def plot_id(graph, lane_id):
    """plot topology graph"""
    plt.close()
    plt.figure()
    lane_middle_point_map = {}
    plot_ids = [lane_id]
    for eg in graph.edge:
        if eg.from_lane_id == lane_id:
            plot_ids.append(eg.to_lane_id)
    for i, (nd, color) in enumerate(zip(graph.node, color_iter)):
        if nd.lane_id in plot_ids:
            nd_mid_pt = plot_node(nd, color)
            lane_middle_point_map[nd.lane_id] = nd_mid_pt
    for i, eg in enumerate(graph.edge):
        if eg.from_lane_id == lane_id:
            plot_edge(eg, lane_middle_point_map)
    plt.gca().set_aspect(1)
    plt.title('Routing topology graph')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()

    plt.draw()


def print_help():
    """Print help information.

    Print help information of usage.

    Args:

    """
    print 'usage:'
    print '     python debug_topo.py file_path, then',
    print_help_command()


def print_help_command():
    """Print command help information.

    Print help information of command.

    Args:

    """
    print 'type in command: [q] [a] [i lane_id]'
    print '         q               exit'
    print '         a               plot all topology'
    print '         i lane_id       plot lanes could be reached from lane with lane_id'


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print_help()
        sys.exit(0)
    print 'Please wait for loading data...'
    file_name = sys.argv[1]
    fin = open(file_name)
    graph = topo_graph_pb2.Graph()
    graph.ParseFromString(fin.read())

    plt.ion()
    while 1:
        print_help_command()
        print 'cmd>',
        instruction = raw_input()
        argv = instruction.strip(' ').split(' ')
        if len(argv) == 1:
            if argv[0] == 'q':
                sys.exit(0)
            elif argv[0] == 'a':
                plot_all(graph)
            else:
                print '[ERROR] wrong command'
            continue

        if len(argv) == 2:
            if argv[0] == 'i':
                plot_id(graph, argv[1])
            else:
                print '[ERROR] wrong command'
            continue

        else:
            print '[ERROR] wrong arguments'
            continue
