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

import sys
sys.path.append("../../bazel-genfiles")
from modules.planning.proto import planning_pb2
from modules.localization.proto import localization_pb2
import matplotlib.pyplot as plt
from google.protobuf import text_format
import mkz_polygon
import gflags
from gflags import FLAGS

def find_closest_t(points_t, current_t):
    if len(points_t) == 0:
        return -1
    if len(points_t) == 1:
        return points_t[0]
    if len(points_t) == 2:
        if abs(points_t[0] - current_t) < abs(points_t[1] - current_t):
            return points_t[0]
        else:
            return points_t[1]
    if points_t[len(points_t) / 2] > current_t:
        return find_closest_t(points_t[0:len(points_t) / 2], current_t)
    elif points_t[len(points_t) / 2] < current_t:
        return find_closest_t(points_t[len(points_t) / 2 + 1:], current_t)
    else:
        return current_t


if len(sys.argv) < 2:
    print "usage: python main.py planning.pb.txt localization.pb.txt"
    sys.exit()

planning_pb_file = sys.argv[1]
planning_pb = planning_pb2.ADCTrajectory()
f_handle = open(planning_pb_file, 'r')
text_format.Merge(f_handle.read(), planning_pb)
f_handle.close()
#print planning_pb_data

localization_pb_file = sys.argv[2]
localization_pb = localization_pb2.LocalizationEstimate()
f_handle = open(localization_pb_file, 'r')
text_format.Merge(f_handle.read(), localization_pb)
f_handle.close()

# plot traj
points_x = []
points_y = []
points_t = []
base_time_sec = planning_pb.header.timestamp_sec
for trajectory_point in planning_pb.adc_trajectory_point:
    points_x.append(trajectory_point.x)
    points_y.append(trajectory_point.y)
    points_t.append(base_time_sec + trajectory_point.relative_time)
plt.plot(points_x, points_y, "r.")

# plot car
loc_x = [localization_pb.pose.position.x]
loc_y = [localization_pb.pose.position.y]
current_t = localization_pb.header.timestamp_sec
plt.plot(loc_x, loc_y, "bo")
position = []
position.append(localization_pb.pose.position.x)
position.append(localization_pb.pose.position.y)
position.append(localization_pb.pose.position.z)
quaternion = []
quaternion.append(localization_pb.pose.orientation.qx)
quaternion.append(localization_pb.pose.orientation.qy)
quaternion.append(localization_pb.pose.orientation.qz)
quaternion.append(localization_pb.pose.orientation.qw)
mkz_polygon.plot(position, quaternion, plt)
content = "t = " + str(current_t) + "\n"
content += "speed @y = " + str(localization_pb.pose.linear_velocity.y) + "\n"
content += "acc @y = " + str(localization_pb.pose.linear_acceleration_vrf.y)
lxy = [-80, 80]
plt.annotate(
    content,
    xy=(loc_x[0], loc_y[0]),
    xytext=lxy,
    textcoords='offset points',
    ha='left',
    va='top',
    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.3),
    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
    alpha=0.8)

# plot projected point
matched_t = find_closest_t(points_t, current_t)
idx = points_t.index(matched_t)
plt.plot([points_x[idx]], [points_y[idx]], "bs")
content = "t = " + str(matched_t) + "\n"
content += "speed = " + str(trajectory_point.speed) + "\n"
content += "acc = " + str(trajectory_point.acceleration_s)
lxy = [-80, -80]
plt.annotate(
    content,
    xy=(points_x[idx], points_y[idx]),
    xytext=lxy,
    textcoords='offset points',
    ha='right',
    va='top',
    bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.3),
    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
    alpha=0.8)

plt.axis('equal')
plt.show()
