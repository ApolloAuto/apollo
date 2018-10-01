from hybrid_a_star_python_interface import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import time
import math


# initialze object
HybridAStar = HybridAStarPlanner()

# parameter(except max, min and car size is defined in proto)
num_output_buffer = 100000
sx = -15.0
sy = 8
sphi = 0.0
ex = 0.0
ey = 0.0
ephi = math.pi / 2

#obstacles(x, y, heading, length, width, id)
HybridAStar.AddVirtualObstacle(0.0, 13.0, 0.0, 40.0, 4.0, 1)
HybridAStar.AddVirtualObstacle(-11, 0.0, 0.0, 18.0, 10.0, 2)
HybridAStar.AddVirtualObstacle(11, 0.0, 0.0, 18.0, 10.0, 3)


x = (c_double * num_output_buffer)()
y = (c_double * num_output_buffer)()
phi = (c_double * num_output_buffer)()
size = (c_ushort * 1)()

start = time.time()
print("planning start")
if not HybridAStar.Plan(sx, sy, sphi, ex, ey, ephi):
    print("planning fail")
    exit()
end = time.time()
print("planning time is " + str(end - start))

# load result
HybridAStar.GetResult(x, y, phi, size)
x_out = []
y_out = []
phi_out = []
for i in range(0, size[0]):
    x_out.append(float(x[i]))
    y_out.append(float(y[i]))
    phi_out.append(float(phi[i]))

# plot
fig = plt.figure(1)
ax = plt.gca()
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
plt.plot(sx, sy, "s")
plt.plot(ex, ey, "s")
rect1 = patches.Rectangle((-20.0, 11.0), 40.0, 4.0, 0.0)
rect2 = patches.Rectangle((-20.0, -5.0), 18.0, 10.0, 0.0)
rect3 = patches.Rectangle((2.0, -5.0), 18.0, 10.0, 0.0)
ax.add_patch(rect1)
ax.add_patch(rect2)
ax.add_patch(rect3)
plt.axis('equal')
plt.show()

