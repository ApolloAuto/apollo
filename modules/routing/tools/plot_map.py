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

import modules.map.proto.map_pb2 as map_pb2
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

    for lane in drivemap.lane:
        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                draw_boundary(ax, curve.line_segment)

        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                draw_boundary(ax, curve.line_segment)
    plt.draw()

    return drivemap
