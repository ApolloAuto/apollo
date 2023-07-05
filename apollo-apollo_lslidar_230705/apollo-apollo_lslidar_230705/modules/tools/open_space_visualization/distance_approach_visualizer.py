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

from distance_approach_python_interface import *


result_file = "/tmp/open_space_osqp_ipopt.csv"


# def SmoothTrajectory(visualize_flag):
def SmoothTrajectory(visualize_flag, sx, sy):
    # initialze object
    OpenSpacePlanner = DistancePlanner()

    # parameter(except max, min and car size is defined in proto)
    num_output_buffer = 10000
    # sx = -8
    # sy = 1.5
    # sphi = 0.5
    sphi = 0.0

    scenario = "backward"
    # scenario = "parallel"

    if scenario == "backward":
        # obstacles for distance approach(vertices coords in clock wise order)
        ROI_distance_approach_parking_boundary = (
            c_double * 20)(*[-13.6407054776,
                             0.0140634663703,
                             0.0,
                             0.0,
                             0.0515703622475,
                             -5.15258191624,
                             0.0515703622475,
                             -5.15258191624,
                             2.8237895441,
                             -5.15306980547,
                             2.8237895441,
                             -5.15306980547,
                             2.7184833539,
                             -0.0398078878812,
                             16.3592013995,
                             -0.011889513383,
                             16.3591910364,
                             5.60414234644,
                             -13.6406951857,
                             5.61797800844,
                             ])
        OpenSpacePlanner.AddObstacle(
            ROI_distance_approach_parking_boundary)
        # parking lot position
        ex = 1.359
        ey = -3.86443643718
        ephi = 1.581
        XYbounds = [-13.6406951857, 16.3591910364, -5.15258191624, 5.61797800844]

    x = (c_double * num_output_buffer)()
    y = (c_double * num_output_buffer)()
    phi = (c_double * num_output_buffer)()
    v = (c_double * num_output_buffer)()
    a = (c_double * num_output_buffer)()
    steer = (c_double * num_output_buffer)()
    opt_x = (c_double * num_output_buffer)()
    opt_y = (c_double * num_output_buffer)()
    opt_phi = (c_double * num_output_buffer)()
    opt_v = (c_double * num_output_buffer)()
    opt_a = (c_double * num_output_buffer)()
    opt_steer = (c_double * num_output_buffer)()
    opt_time = (c_double * num_output_buffer)()
    opt_dual_l = (c_double * num_output_buffer)()
    opt_dual_n = (c_double * num_output_buffer)()
    size = (c_ushort * 1)()
    XYbounds_ctype = (c_double * 4)(*XYbounds)
    hybrid_time = (c_double * 1)(0.0)
    dual_time = (c_double * 1)(0.0)
    ipopt_time = (c_double * 1)(0.0)

    success = True
    start = time.time()
    print("planning start")
    if not OpenSpacePlanner.DistancePlan(sx, sy, sphi, ex, ey, ephi, XYbounds_ctype):
        print("planning fail")
        success = False
    #   exit()
    planning_time = time.time() - start
    print("planning time is " + str(planning_time))

    x_out = []
    y_out = []
    phi_out = []
    v_out = []
    a_out = []
    steer_out = []
    opt_x_out = []
    opt_y_out = []
    opt_phi_out = []
    opt_v_out = []
    opt_a_out = []
    opt_steer_out = []
    opt_time_out = []
    opt_dual_l_out = []
    opt_dual_n_out = []

    if visualize_flag and success:
        # load result
        OpenSpacePlanner.DistanceGetResult(x, y, phi, v, a, steer, opt_x,
                                           opt_y, opt_phi, opt_v, opt_a, opt_steer, opt_time,
                                           opt_dual_l, opt_dual_n, size,
                                           hybrid_time, dual_time, ipopt_time)
        for i in range(0, size[0]):
            x_out.append(float(x[i]))
            y_out.append(float(y[i]))
            phi_out.append(float(phi[i]))
            v_out.append(float(v[i]))
            a_out.append(float(a[i]))
            steer_out.append(float(steer[i]))
            opt_x_out.append(float(opt_x[i]))
            opt_y_out.append(float(opt_y[i]))
            opt_phi_out.append(float(opt_phi[i]))
            opt_v_out.append(float(opt_v[i]))
            opt_a_out.append(float(opt_a[i]))
            opt_steer_out.append(float(opt_steer[i]))
            opt_time_out.append(float(opt_time[i]))

        for i in range(0, size[0] * 6):
            opt_dual_l_out.append(float(opt_dual_l[i]))
        for i in range(0, size[0] * 16):
            opt_dual_n_out.append(float(opt_dual_n[i]))
        # trajectories plot
        fig1 = plt.figure(1)
        ax = fig1.add_subplot(111)
        for i in range(0, size[0]):
            # warm start
            downx = 1.055 * math.cos(phi_out[i] - math.pi / 2)
            downy = 1.055 * math.sin(phi_out[i] - math.pi / 2)
            leftx = 1.043 * math.cos(phi_out[i] - math.pi)
            lefty = 1.043 * math.sin(phi_out[i] - math.pi)
            x_shift_leftbottom = x_out[i] + downx + leftx
            y_shift_leftbottom = y_out[i] + downy + lefty
            warm_start_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055 * 2,
                                               angle=phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='r', facecolor='none')
            warm_start_arrow = patches.Arrow(
                x_out[i], y_out[i], 0.25 * math.cos(phi_out[i]), 0.25 * math.sin(phi_out[i]), 0.2, edgecolor='r',)
            # ax.add_patch(warm_start_car)
            ax.add_patch(warm_start_arrow)
            # distance approach
            downx = 1.055 * math.cos(opt_phi_out[i] - math.pi / 2)
            downy = 1.055 * math.sin(opt_phi_out[i] - math.pi / 2)
            leftx = 1.043 * math.cos(opt_phi_out[i] - math.pi)
            lefty = 1.043 * math.sin(opt_phi_out[i] - math.pi)
            x_shift_leftbottom = opt_x_out[i] + downx + leftx
            y_shift_leftbottom = opt_y_out[i] + downy + lefty
            smoothing_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055 * 2,
                                              angle=opt_phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='y', facecolor='none')
            smoothing_arrow = patches.Arrow(
                opt_x_out[i], opt_y_out[i], 0.25 * math.cos(opt_phi_out[i]), 0.25 * math.sin(opt_phi_out[i]), 0.2, edgecolor='y',)
            ax.add_patch(smoothing_car)
            ax.add_patch(smoothing_arrow)

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

        # input plot
        fig2 = plt.figure(2)
        v_graph = fig2.add_subplot(411)
        v_graph.title.set_text('v')
        v_graph.plot(np.linspace(0, size[0], size[0]), v_out)
        v_graph.plot(np.linspace(0, size[0], size[0]), opt_v_out)
        a_graph = fig2.add_subplot(412)
        a_graph.title.set_text('a')
        a_graph.plot(np.linspace(0, size[0], size[0]), a_out)
        a_graph.plot(np.linspace(0, size[0], size[0]), opt_a_out)
        steer_graph = fig2.add_subplot(413)
        steer_graph.title.set_text('steering')
        steer_graph.plot(np.linspace(0, size[0], size[0]), steer_out)
        steer_graph.plot(np.linspace(0, size[0], size[0]), opt_steer_out)
        steer_graph = fig2.add_subplot(414)
        steer_graph.title.set_text('t')
        steer_graph.plot(np.linspace(0, size[0], size[0]), opt_time_out)
        # dual variables
        fig3 = plt.figure(3)
        dual_l_graph = fig3.add_subplot(211)
        dual_l_graph.title.set_text('dual_l')
        dual_l_graph.plot(np.linspace(0, size[0] * 6, size[0] * 6), opt_dual_l_out)
        dual_n_graph = fig3.add_subplot(212)
        dual_n_graph.title.set_text('dual_n')
        dual_n_graph.plot(np.linspace(0, size[0] * 16, size[0] * 16), opt_dual_n_out)
        plt.show()
        return True

    if not visualize_flag:
        if success:
            # load result
            OpenSpacePlanner.DistanceGetResult(x, y, phi, v, a, steer, opt_x,
                                               opt_y, opt_phi, opt_v, opt_a, opt_steer, opt_time,
                                               opt_dual_l, opt_dual_n, size,
                                               hybrid_time, dual_time, ipopt_time)
            for i in range(0, size[0]):
                x_out.append(float(x[i]))
                y_out.append(float(y[i]))
                phi_out.append(float(phi[i]))
                v_out.append(float(v[i]))
                a_out.append(float(a[i]))
                steer_out.append(float(steer[i]))
                opt_x_out.append(float(opt_x[i]))
                opt_y_out.append(float(opt_y[i]))
                opt_phi_out.append(float(opt_phi[i]))
                opt_v_out.append(float(opt_v[i]))
                opt_a_out.append(float(opt_a[i]))
                opt_steer_out.append(float(opt_steer[i]))
                opt_time_out.append(float(opt_time[i]))
            # check end_pose distacne
            end_pose_dist = math.sqrt((opt_x_out[-1] - ex)**2 + (opt_y_out[-1] - ey)**2)
            end_pose_heading = abs(opt_phi_out[-1] - ephi)
            reach_end_pose = (end_pose_dist <= 0.1 and end_pose_heading <= 0.17)
        else:
            end_pose_dist = 100.0
            end_pose_heading = 100.0
            reach_end_pose = 0
        return [success, end_pose_dist, end_pose_heading, reach_end_pose, opt_x_out, opt_y_out, opt_phi_out, opt_v_out, opt_a_out, opt_steer_out, opt_time_out,
                hybrid_time, dual_time, ipopt_time, planning_time]
    return False


if __name__ == '__main__':
    # visualize_flag = True
    # SmoothTrajectory(visualize_flag)

    visualize_flag = False
    planning_time_stats = []
    hybrid_time_stats = []
    dual_time_stats = []
    ipopt_time_stats = []
    end_pose_dist_stats = []
    end_pose_heading_stats = []

    test_count = 0
    success_count = 0
    for sx in np.arange(-10, 10, 1.0):
        for sy in np.arange(2, 4, 0.5):
            print("sx is " + str(sx) + " and sy is " + str(sy))
            test_count += 1
            result = SmoothTrajectory(visualize_flag, sx, sy)
            # if result[0] and result[3]:  # success cases only
            if result[0]:
                success_count += 1
                planning_time_stats.append(result[-1])
                ipopt_time_stats.append(result[-2][0])
                dual_time_stats.append(result[-3][0])
                hybrid_time_stats.append(result[-4][0])
                end_pose_dist_stats.append(result[1])
                end_pose_heading_stats.append(result[2])

    print("success rate is " + str(float(success_count) / float(test_count)))
    print("min is " + str(min(planning_time_stats)))
    print("max is " + str(max(planning_time_stats)))
    print("average is " + str(sum(planning_time_stats) / len(planning_time_stats)))
    print("max end_pose_dist difference is: " + str(max(end_pose_dist_stats)))
    print("min end_pose_dist difference is: " + str(min(end_pose_dist_stats)))
    print("average end_pose_dist difference is: " +
          str(sum(end_pose_dist_stats) / len(end_pose_dist_stats)))
    print("max end_pose_heading difference is: " + str(max(end_pose_heading_stats)))
    print("min end_pose_heading difference is: " + str(min(end_pose_heading_stats)))
    print("average end_pose_heading difference is: " +
          str(sum(end_pose_heading_stats) / len(end_pose_heading_stats)))

    module_timing = np.asarray([hybrid_time_stats, dual_time_stats, ipopt_time_stats])
    np.savetxt(result_file, module_timing, delimiter=",")

    print("average hybrid time(s): %4.4f, with max: %4.4f, min: %4.4f" % (
        sum(hybrid_time_stats) / len(hybrid_time_stats) / 1000.0, max(hybrid_time_stats) / 1000.0,
        min(hybrid_time_stats) / 1000.0))
    print("average dual time(s): %4.4f, with max: %4.4f, min: %4.4f" % (
        sum(dual_time_stats) / len(dual_time_stats) / 1000.0, max(dual_time_stats) / 1000.0,
        min(dual_time_stats) / 1000.0))
    print("average ipopt time(s): %4.4f, with max: %4.4f, min: %4.4f" % (
        sum(ipopt_time_stats) / len(ipopt_time_stats) / 1000.0, max(ipopt_time_stats) / 1000.0,
        min(ipopt_time_stats) / 1000.0))
