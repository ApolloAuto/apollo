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

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from modules.map.relative_map.proto import navigation_pb2

ax = None
map_msg = None


def draw_lane_boundary(lane, ax, color_val):
    """draw boundary"""
    for curve in lane.left_boundary.curve.segment:
        if curve.HasField('line_segment'):
            px = []
            py = []
            for p in curve.line_segment.point:
                px.append(float(p.y))
                py.append(float(p.x))
            ax.plot(px, py, ls='-', c=color_val, alpha=0.5)
    for curve in lane.right_boundary.curve.segment:
        if curve.HasField('line_segment'):
            px = []
            py = []
            for p in curve.line_segment.point:
                px.append(float(p.y))
                py.append(float(p.x))
            ax.plot(px, py, ls='-', c=color_val, alpha=0.5)


def draw_lane_central(lane, ax, color_val):
    """draw boundary"""
    for curve in lane.central_curve.segment:
        if curve.HasField('line_segment'):
            px = []
            py = []
            for p in curve.line_segment.point:
                px.append(float(p.y))
                py.append(float(p.x))
            ax.plot(px, py, ls=':', c=color_val, alpha=0.5)


def update(frame_number):
    plt.cla()
    if map_msg is not None:
        for lane in map_msg.hdmap.lane:
            draw_lane_boundary(lane, ax, 'b')
            draw_lane_central(lane, ax, 'r')

        for key in map_msg.navigation_path:
            x = []
            y = []
            for point in map_msg.navigation_path[key].path.path_point:
                x.append(point.y)
                y.append(point.x)
            ax.plot(x, y, ls='-', c='g', alpha=0.3)

    ax.axvline(x=0.0, alpha=0.3)
    ax.axhline(y=0.0, alpha=0.3)
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 200])
    ax.relim()


def map_callback(map_msg_pb):
    global map_msg
    map_msg = map_msg_pb


def add_listener():
    rospy.init_node('relative_map_plot', anonymous=True)
    rospy.Subscriber('/apollo/relative_map',
                     navigation_pb2.MapMsg,
                     map_callback)


if __name__ == '__main__':
    add_listener()
    fig = plt.figure()
    ax = plt.subplot2grid((1, 1), (0, 0), rowspan=3, colspan=1)
    ani = animation.FuncAnimation(fig, update, interval=100)
    plt.show()
