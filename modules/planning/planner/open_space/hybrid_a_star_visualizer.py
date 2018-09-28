from hybrid_a_star_python_interface import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import math


#initialze object
HybridAStar = HybridAStarPlanner()

#parameter(except max, min and car size is defined in proto) 
num_output_buffer = 100000
sx = 0.0
sy = 0.0
sphi = 0.0
ex = 100.0
ey = 100.0
ephi = math.pi
#obstacles(x, y, heading, length, width)
HybridAStar.AddVirtualObstacle(50.0, 50.0, 0.0, 20.0, 20.0)


x = (c_double * num_output_buffer)()
y = (c_double * num_output_buffer)()
phi = (c_double * num_output_buffer)()
size = (c_ushort * 1)()

start = time.time()
if not HybridAStar.Plan(sx,sy,sphi,ex,ey,ephi) :
    print("planning fail")
    exit()
end = time.time()
print("planning time is " +str(end - start))

#load result
HybridAStar.GetResult(x, y, phi, size);
x_out = []
y_out = []
phi_out = []
for i in range(0, size[0]):
    x_out.append(float(x[i]))
    y_out.append(float(y[i]))
    phi_out.append(float(phi[i]))

#plot
plt.figure(1)
ax = plt.gca()
for i in range(0, size[0]):
    arrow = patches.Arrow(x_out[i], y_out[i], 0.25*math.cos(phi_out[i]), 0.25*math.sin(phi_out[i]))
    ax.add_patch(arrow)
#plt.plot(x_out, y_out, "o")
plt.plot(sx, sy, "s")
plt.plot(ex, ey, "s")
rect = patches.Rectangle((40.0, 40.0),20.0,20.0,0.0)
ax.add_patch(rect)
plt.show()

