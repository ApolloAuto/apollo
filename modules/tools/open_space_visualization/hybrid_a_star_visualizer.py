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

import math
import time

from matplotlib import animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from hybrid_a_star_python_interface import *


def HybridAStarPlan(visualize_flag):
    # initialze object
    HybridAStar = HybridAStarPlanner()

    # parameter(except max, min and car size is defined in proto)
    num_output_buffer = 100000
    sx = -8
    sy = 4
    sphi = 0.0

    scenario = "backward"
    # scenario = "parallel"

    if scenario == "backward":
        # for parking space 11543 in sunnyvale_with_two_offices
        left_boundary_x = (
            c_double * 3)(*[-13.6407054776, 0.0, 0.0515703622475])
        left_boundary_y = (
            c_double * 3)(*[0.0140634663703, 0.0, -5.15258191624])
        down_boundary_x = (c_double * 2)(*[0.0515703622475, 2.8237895441])
        down_boundary_y = (c_double * 2)(*[-5.15258191624, -5.15306980547])
        right_boundary_x = (
            c_double * 3)(*[2.8237895441, 2.7184833539, 16.3592013995])
        right_boundary_y = (
            c_double * 3)(*[-5.15306980547, -0.0398078878812, -0.011889513383])
        up_boundary_x = (c_double * 2)(*[16.3591910364, -13.6406951857])
        up_boundary_y = (c_double * 2)(*[5.60414234644, 5.61797800844])
        # obstacles(x, y, size)
        HybridAStar.AddVirtualObstacle(left_boundary_x, left_boundary_y, 3)
        HybridAStar.AddVirtualObstacle(
            down_boundary_x, down_boundary_y, 2)
        HybridAStar.AddVirtualObstacle(
            right_boundary_x, right_boundary_y, 3)
        HybridAStar.AddVirtualObstacle(
            up_boundary_x, up_boundary_y, 2)
        ex = 1.359
        ey = -3.86443643718
        ephi = 1.581
        XYbounds = [-13.6406951857, 16.3591910364, -
                    5.15258191624, 5.61797800844]

    x = (c_double * num_output_buffer)()
    y = (c_double * num_output_buffer)()
    phi = (c_double * num_output_buffer)()
    v = (c_double * num_output_buffer)()
    a = (c_double * num_output_buffer)()
    steer = (c_double * num_output_buffer)()
    size = (c_ushort * 1)()
    XYbounds_ctype = (c_double * 4)(*XYbounds)

    start = time.time()
    print("planning start")
    success = True
    if not HybridAStar.Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_ctype):
        print("planning fail")
        success = False
    end = time.time()
    planning_time = end - start
    print("planning time is " + str(planning_time))

    # load result
    x_out = []
    y_out = []
    phi_out = []
    v_out = []
    a_out = []
    steer_out = []

    if visualize_flag and success:
        HybridAStar.GetResult(x, y, phi, v, a, steer, size)
        for i in range(0, size[0]):
            x_out.append(float(x[i]))
            y_out.append(float(y[i]))
            phi_out.append(float(phi[i]))
            v_out.append(float(v[i]))
            a_out.append(float(a[i]))
            steer_out.append(float(steer[i]))

        # plot
        fig1 = plt.figure(1)
        ax = fig1.add_subplot(111)
        for i in range(0, size[0]):
            downx = 1.055 * math.cos(phi_out[i] - math.pi / 2)
            downy = 1.055 * math.sin(phi_out[i] - math.pi / 2)
            leftx = 1.043 * math.cos(phi_out[i] - math.pi)
            lefty = 1.043 * math.sin(phi_out[i] - math.pi)
            x_shift_leftbottom = x_out[i] + downx + leftx
            y_shift_leftbottom = y_out[i] + downy + lefty
            car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055*2,
                                    angle=phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='r', facecolor='none')
            arrow = patches.Arrow(
                x_out[i], y_out[i], 0.25*math.cos(phi_out[i]), 0.25*math.sin(phi_out[i]), 0.2)
            ax.add_patch(car)
            ax.add_patch(arrow)
        ax.plot(sx, sy, "s")
        ax.plot(ex, ey, "s")
        if scenario == "backward":
            left_boundary_x = [-13.6407054776, 0.0, 0.0515703622475]
            left_boundary_y = [0.0140634663703, 0.0, -5.15258191624]
            down_boundary_x = [0.0515703622475, 2.8237895441]
            down_boundary_y = [-5.15258191624, -5.15306980547]
            right_boundary_x = [2.8237895441, 2.7184833539, 16.3592013995]
            right_boundary_y = [-5.15306980547, -0.0398078878812, -0.011889513383]
            up_boundary_x = [16.3591910364, -13.6406951857]
            up_boundary_y = [5.60414234644, 5.61797800844]
            ax.plot(left_boundary_x, left_boundary_y, "k")
            ax.plot(down_boundary_x, down_boundary_y, "k")
            ax.plot(right_boundary_x, right_boundary_y, "k")
            ax.plot(up_boundary_x, up_boundary_y, "k")

        plt.axis('equal')

        fig2 = plt.figure(2)
        v_graph = fig2.add_subplot(311)
        v_graph.title.set_text('v')
        v_graph.plot(np.linspace(0, size[0], size[0]), v_out)
        a_graph = fig2.add_subplot(312)
        a_graph.title.set_text('a')
        a_graph.plot(np.linspace(0, size[0], size[0]), a_out)
        steer_graph = fig2.add_subplot(313)
        steer_graph.title.set_text('steering')
        steer_graph.plot(np.linspace(0, size[0], size[0]), steer_out)
        plt.show()
    if not visualize_flag:
        if success:
            HybridAStar.GetResult(x, y, phi, v, a, steer, size)
            for i in range(0, size[0]):
                x_out.append(float(x[i]))
                y_out.append(float(y[i]))
                phi_out.append(float(phi[i]))
                v_out.append(float(v[i]))
                a_out.append(float(a[i]))
                steer_out.append(float(steer[i]))
        return success, x_out, y_out, phi_out, v_out, a_out, steer_out, planning_time


if __name__ == '__main__':
    visualize_flag = True
    HybridAStarPlan(visualize_flag)
