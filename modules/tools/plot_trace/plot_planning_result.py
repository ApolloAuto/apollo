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

from subprocess import call
import sys

from google.protobuf import text_format
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2
from modules.planning.proto import planning_pb2


g_args = None


def get_3d_trajectory(planning_pb):
    x = [p.path_point.x for p in planning_pb.trajectory_point]
    y = [p.path_point.y for p in planning_pb.trajectory_point]
    z = [p.v for p in planning_pb.trajectory_point]
    return (x, y, z)


def get_debug_paths(planning_pb):
    if not planning_pb.HasField("debug"):
        return None
    if not planning_pb.debug.HasField("planning_data"):
        return None
    results = []
    for path in planning_pb.debug.planning_data.path:
        x = [p.x for p in path.path_point]
        y = [p.y for p in path.path_point]
        results.append((path.name, (x, y)))
    return results


def plot_planning(ax, planning_file):
    with open(planning_file, 'r') as fp:
        planning_pb = planning_pb2.ADCTrajectory()
        text_format.Merge(fp.read(), planning_pb)
        trajectory = get_3d_trajectory(planning_pb)
        ax.plot(trajectory[0], trajectory[1], trajectory[2],
                label="Trajectory:%s" % planning_file)
        paths = get_debug_paths(planning_pb)
        if paths:
            for name, path in paths:
                ax.plot(path[0], path[1], label="%s:%s" % (name, planning_file))
        ax.legend()


def press_key(event):
    if event.key == 'c':
        files = g_args.planning_files
        if len(files) != 2:
            print('Need more than two files.')
            return
        command = ["cp"]
        for f in files:
            command.append(f)
        if call(command) == 0:
            print('command success: %s' % " ".join(command))
            sys.exit(0)
        else:
            print('Failed to run command: %s ' % " ".join(command))
            sys.exit(1)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description="""A visualization tool that can plot one or multiple planning "
        results, so that we can compare the differences.
        Example: plot_planning_result.py result_file1.pb.txt result_file2.pb.txt"""
    )
    parser.add_argument(
        "planning_files",
        action='store',
        nargs="+",
        help="The planning results")
    parser.add_argument(
        "--figure",
        action='store',
        type=str,
        help="Save the planning results to picture, if not set, show on screen"
    )
    g_args = parser.parse_args()

    matplotlib.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    fig.canvas.mpl_connect('key_press_event', press_key)
    ax = fig.gca(projection='3d')
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("speed")

    for planning_file in g_args.planning_files:
        plot_planning(ax, planning_file)

    if g_args.figure:
        plt.savefig(g_args.figure)
        print('picture saved to %s' % g_args.figure)
    else:
        plt.show()
