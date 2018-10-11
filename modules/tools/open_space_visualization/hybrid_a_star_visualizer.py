from hybrid_a_star_python_interface import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import numpy as np
import time
import math


# initialze object
HybridAStar = HybridAStarPlanner()

# parameter(except max, min and car size is defined in proto)
num_output_buffer = 100000
sx = -15.0
sy = 7
sphi = 0.0

scenario = "backward"
# scenario = "parallel"

if scenario == "backward":
    #obstacles(x, y, heading, length, width, id)
    HybridAStar.AddVirtualObstacle(0.0, 13.0, 0.0, 40.0, 4.0, 1)
    HybridAStar.AddVirtualObstacle(-11, 0.0, 0.0, 18.0, 10.0, 2)
    HybridAStar.AddVirtualObstacle(11, 0.0, 0.0, 18.0, 10.0, 3)
    ex = 0.0
    ey = 0.0
    ephi = math.pi / 2
elif scenario == "parallel":
    #obstacles(x, y, heading, length, width, id)
    HybridAStar.AddVirtualObstacle(0.0, 13.0, 0.0, 40.0, 4.0, 1)
    HybridAStar.AddVirtualObstacle(-12, 0.0, 0.0, 16.0, 10.0, 2)
    HybridAStar.AddVirtualObstacle(12, 0.0, 0.0, 16.0, 10.0, 3)
    HybridAStar.AddVirtualObstacle(0.0, -1.25, 0.0, 40.0, 7.5, 4)
    ex = -1.75
    ey = 4.0
    ephi = 0


x = (c_double * num_output_buffer)()
y = (c_double * num_output_buffer)()
phi = (c_double * num_output_buffer)()
v = (c_double * num_output_buffer)()
a = (c_double * num_output_buffer)()
steer = (c_double * num_output_buffer)()
size = (c_ushort * 1)()

start = time.time()
print("planning start")
if not HybridAStar.Plan(sx, sy, sphi, ex, ey, ephi):
    print("planning fail")
    exit()
end = time.time()
print("planning time is " + str(end - start))

# load result
HybridAStar.GetResult(x, y, phi, v, a, steer, size)
x_out = []
y_out = []
phi_out = []
v_out = []
a_out = []
steer_out = []
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
    rect1 = patches.Rectangle((-20.0, 11.0), 40.0, 4.0, 0.0)
    rect2 = patches.Rectangle((-20.0, -5.0), 18.0, 10.0, 0.0)
    rect3 = patches.Rectangle((2.0, -5.0), 18.0, 10.0, 0.0)
    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(rect3)
elif scenario == "parallel":
    rect1 = patches.Rectangle((-20.0, 11.0), 40.0, 4.0, 0.0)
    rect2 = patches.Rectangle((-20.0, -5.0), 16.0, 10.0, 0.0)
    rect3 = patches.Rectangle((4.0, -5.0), 16.0, 10.0, 0.0)
    rect4 = patches.Rectangle((-20.0, -5.0), 40.0, 7.5, 0.0)
    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(rect3)
    ax.add_patch(rect4)
plt.axis('equal')

fig2 = plt.figure(2)
v_graph = fig2.add_subplot(311);
v_graph.plot(np.linspace(0, size[0], size[0]), v_out)
a_graph = fig2.add_subplot(312);
a_graph.plot(np.linspace(0, size[0], size[0]), a_out)
steer_graph = fig2.add_subplot(313);
steer_graph.plot(np.linspace(0, size[0], size[0]), steer_out)
plt.show()
