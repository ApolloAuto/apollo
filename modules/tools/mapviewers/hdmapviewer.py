#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

import argparse

from bokeh.plotting import figure, output_file, show
from modules.common_msgs.map_msgs import map_pb2
import modules.tools.common.proto_utils as proto_utils


def draw(map_pb, plot):
    for lane in map_pb.lane:
        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                x = []
                y = []
                for p in curve.line_segment.point:
                    x.append(p.x)
                    y.append(p.y)
                plot.line(x, y, line_width=2)
        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                x = []
                y = []
                for p in curve.line_segment.point:
                    x.append(p.x)
                    y.append(p.y)
                plot.line(x, y, line_width=2)


def load_map_data(map_file):
    map_pb = map_pb2.Map()
    proto_utils.get_pb_from_file(map_file, map_pb)
    return map_pb


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="HDMapViewer is a tool to display hdmap.",
        prog="hdmapviewer.py")

    parser.add_argument(
        "-m", "--map", action="store", type=str, required=True,
        help="Specify the HDMap file in txt or binary format")

    args = parser.parse_args()
    map_pb = load_map_data(args.map)

    output_file("hdmap.html")
    plot = figure(sizing_mode='scale_both', match_aspect=True)

    draw(map_pb, plot)
    show(plot)
