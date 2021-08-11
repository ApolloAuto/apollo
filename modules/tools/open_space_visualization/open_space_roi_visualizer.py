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


# @file to run it, change the modules/common/configs/config_gflags.cc to use sunnyvale_with_two_offices


from open_space_roi_interface import *
import matplotlib.pyplot as plt

# initialize object
open_space_roi = open_space_roi()


lane_id = "11564dup1_1_-1"
parking_id = "11543"
num_output_buffer = 50
unrotated_roi_boundary_x = (c_double * num_output_buffer)()
roi_boundary_x = (c_double * num_output_buffer)()
parking_spot_x = (c_double * num_output_buffer)()
unrotated_roi_boundary_y = (c_double * num_output_buffer)()
roi_boundary_y = (c_double * num_output_buffer)()
parking_spot_y = (c_double * num_output_buffer)()
end_pose = (c_double * num_output_buffer)()
xy_boundary = (c_double * num_output_buffer)()
origin_pose = (c_double * num_output_buffer)()

if not open_space_roi.ROITest(lane_id, parking_id,
                              unrotated_roi_boundary_x, unrotated_roi_boundary_y, roi_boundary_x, roi_boundary_y,
                              parking_spot_x, parking_spot_y, end_pose,
                              xy_boundary, origin_pose):
    print("open_space_roi fail")
result_unrotated_roi_boundary_x = []
result_unrotated_roi_boundary_y = []
result_roi_boundary_x = []
result_roi_boundary_y = []
result_parking_spot_x = []
result_parking_spot_y = []
result_end_pose = []
result_xy_boundary = []
result_origin_pose = []

print("vertices of obstacles")
for i in range(0, 10):
    result_unrotated_roi_boundary_x.append(float(unrotated_roi_boundary_x[i]))
    result_unrotated_roi_boundary_y.append(float(unrotated_roi_boundary_y[i]))
    result_roi_boundary_x.append(float(roi_boundary_x[i]))
    result_roi_boundary_y.append(float(roi_boundary_y[i]))
    print(str(float(roi_boundary_x[i])))
    print(str(float(roi_boundary_y[i])))
print("parking spot")
for i in range(0, 4):
    result_parking_spot_x.append(float(parking_spot_x[i]))
    result_parking_spot_y.append(float(parking_spot_y[i]))
print("end_pose in x,y,phi,v")
for i in range(0, 4):
    print(str(float(end_pose[i])))
print("xy_boundary in xmin xmax ymin ymax")
for i in range(0, 4):
    print(str(float(xy_boundary[i])))
print("origin_pose")
for i in range(0, 2):
    print(str(float(origin_pose[i])))

fig = plt.figure()
ax1 = fig.add_subplot(211)
ax1.scatter(result_unrotated_roi_boundary_x, result_unrotated_roi_boundary_y)
ax1.scatter(result_parking_spot_x, result_parking_spot_y)
ax2 = fig.add_subplot(212)
ax2.scatter(result_roi_boundary_x, result_roi_boundary_y)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
