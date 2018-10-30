#!/usr/bin/env python

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

from distance_approach_python_interface import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import numpy as np
import time
import math


# initialze object
OpenSpacePlanner = DistancePlanner()

# parameter(except max, min and car size is defined in proto)
num_output_buffer = 100000
sx = -15.0
sy = 7
sphi = 0.0

scenario = "backward"
# scenario = "parallel"

if scenario == "backward":
    # obstacles for warm start(x, y, heading, length, width, id)
    OpenSpacePlanner.AddWarmStartObstacle(0.0, 11.5, 0.0, 40.0, 1.0, 1)
    OpenSpacePlanner.AddWarmStartObstacle(-11, 2.5, 0.0, 18.0, 5.0, 2)
    OpenSpacePlanner.AddWarmStartObstacle(11, 2.5, 0.0, 18.0, 5.0, 3)
    OpenSpacePlanner.AddWarmStartObstacle(0.0, -0.5, 0.0, 4.0, 1.0, 4)
    # obstacles for distance approach(vertices coords in clock wise order)
    ROI_distance_approach_parking_boundary = (
        c_double * 20)(*[-20, 5, -2, 5, -2, 0, -2, 0, 2, 0, 2, 0, 2, 5, 20, 5, 20, 11, -20, 11])
    # ROI_distance_approach_parking_boundary = [-20, 5, -
    #                                           2, 5, -2, 0, 2, 0, 2, 5, 20, 5, 20, 11, -20, 11]
    OpenSpacePlanner.AddDistanceApproachObstacle(
        ROI_distance_approach_parking_boundary)
    ex = 0.0
    ey = 2.0
    ephi = math.pi / 2
    XYbounds = [-20, 20, -5, 15]
# elif scenario == "parallel":
#     #obstacles(x, y, heading, length, width, id)
#     OpenSpacePlanner.AddWarmStartObstacle(0.0, 13.0, 0.0, 40.0, 4.0, 1)
#     OpenSpacePlanner.AddWarmStartObstacle(-12, 0.0, 0.0, 16.0, 10.0, 2)
#     OpenSpacePlanner.AddWarmStartObstacle(12, 0.0, 0.0, 16.0, 10.0, 3)
#     OpenSpacePlanner.AddWarmStartObstacle(0.0, -1.25, 0.0, 40.0, 7.5, 4)
#     ex = -1.75
#     ey = 4.0
#     ephi = 0
    # XYbounds = [-20, 20, -20, 20]


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
size = (c_ushort * 1)()
XYbounds_ctype = (c_double * 4)(*XYbounds)

start = time.time()
print("planning start")
if not OpenSpacePlanner.DistancePlan(sx, sy, sphi, ex, ey, ephi, XYbounds_ctype):
    print("planning fail")
    exit()
end = time.time()
print("planning time is " + str(end - start))

# load result
OpenSpacePlanner.DistanceGetResult(x, y, phi, v, a, steer, opt_x,
                                   opt_y, opt_phi, opt_v, opt_a, opt_steer, opt_time, size)
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
    warm_start_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055*2,
                                       angle=phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='r', facecolor='none')
    warm_start_arrow = patches.Arrow(
        x_out[i], y_out[i], 0.25*math.cos(phi_out[i]), 0.25*math.sin(phi_out[i]), 0.2, edgecolor='r',)
    # ax.add_patch(warm_start_car)
    ax.add_patch(warm_start_arrow)
    # distance approach
    downx = 1.055 * math.cos(opt_phi_out[i] - math.pi / 2)
    downy = 1.055 * math.sin(opt_phi_out[i] - math.pi / 2)
    leftx = 1.043 * math.cos(opt_phi_out[i] - math.pi)
    lefty = 1.043 * math.sin(opt_phi_out[i] - math.pi)
    x_shift_leftbottom = opt_x_out[i] + downx + leftx
    y_shift_leftbottom = opt_y_out[i] + downy + lefty
    smoothing_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055*2,
                                      angle=opt_phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='y', facecolor='none')
    smoothing_arrow = patches.Arrow(
        opt_x_out[i], opt_y_out[i], 0.25*math.cos(opt_phi_out[i]), 0.25*math.sin(opt_phi_out[i]), 0.2, edgecolor='y',)
    # ax.add_patch(smoothing_car)
    ax.add_patch(smoothing_arrow)

ax.plot(sx, sy, "s")
ax.plot(ex, ey, "s")
if scenario == "backward":
    rect1 = patches.Rectangle((-20.0, 11.0), 40.0, 1.0, 0.0)
    rect2 = patches.Rectangle((-20.0, 0.0), 18.0, 5.0, 0.0)
    rect3 = patches.Rectangle((2.0, 0.0), 18.0, 5.0, 0.0)
    rect4 = patches.Rectangle((-2.0, -1.0), 4.0, 1.0, 0.0)
    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(rect3)
    ax.add_patch(rect4)
# elif scenario == "parallel":
#     rect1 = patches.Rectangle((-20.0, 11.0), 40.0, 4.0, 0.0)
#     rect2 = patches.Rectangle((-20.0, -5.0), 16.0, 10.0, 0.0)
#     rect3 = patches.Rectangle((4.0, -5.0), 16.0, 10.0, 0.0)
#     rect4 = patches.Rectangle((-20.0, -5.0), 40.0, 7.5, 0.0)
#     ax.add_patch(rect1)
#     ax.add_patch(rect2)
#     ax.add_patch(rect3)
#     ax.add_patch(rect4)
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
plt.show()
