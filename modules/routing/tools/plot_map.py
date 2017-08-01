################################################################################
#
# Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
#
################################################################################
"""
This module provides draw map functions, originally designed by baidu/adu/decision.

Authors: fuxiaoxin(fuxiaoxin@baidu.com)
"""

from gen import map_pb2
import matplotlib.pyplot as plt


def load_pb_from_file(filename, pb_value):
    """load pb from file"""
    try:
        f_handle = open(filename, "rb")
        pb_value.ParseFromString(f_handle.read())
        f_handle.close()
    except Exception as e:
        f_handle = open(filename, 'r')
        text_format.Merge(f_handle.read(), pb_value)
        f_handle.close()
    return pb_value


def downsample_array(array):
    """down sample given array"""
    skip = 5
    result = array[::skip]
    result.append(array[-1])
    return result


def draw_boundary(ax, line_segment):
    """
    :param line_segment:
    :return:
    """
    px = []
    py = []
    for p in line_segment.point:
        px.append(float(p.x))
        py.append(float(p.y))
    px = downsample_array(px)
    py = downsample_array(py)
    ax.plot(px, py, 'k', lw=0.4)


def draw_map(ax, mapfile):
    """ draw map from mapfile"""
    drivemap = load_pb_from_file(mapfile, map_pb2.Map())

    for lane in  drivemap.lane:
        for curve in lane.left_boundary.curve.segment:
            if  curve.HasField('line_segment'):
                draw_boundary(ax, curve.line_segment)

        for curve in lane.right_boundary.curve.segment:
            if  curve.HasField('line_segment'):
                draw_boundary(ax, curve.line_segment)

    return drivemap
