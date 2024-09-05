#!/usr/bin/env python

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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
"""
Control Planning Analyzer
"""
import argparse
import math

import matplotlib.pyplot as plt
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3.record import RecordReader

from modules.common_msgs.localization_msgs import localization_pb2
from modules.common_msgs.perception_msgs import perception_obstacle_pb2

def calculate_corners(x, y, angle_rad, is_rear_axle_point=True):

    front_edge_to_center = 2.151
    length = 2.685
    width = 1.01
    if is_rear_axle_point:
        rear_axle_to_center = front_edge_to_center - length / 2.0
        x += rear_axle_to_center * math.cos(angle_rad)
        y += rear_axle_to_center * math.sin(angle_rad)

    # Calculate right front corner coordinate
    front_right_x = x + (length / 2) * math.cos(angle_rad) + (width / 2) * math.sin(angle_rad)
    front_right_y = y + (length / 2) * math.sin(angle_rad) - (width / 2) * math.cos(angle_rad)

    # Calculate right rear corner coordinate
    rear_right_x = x - (length / 2) * math.cos(angle_rad) + (width / 2) * math.sin(angle_rad)
    rear_right_y = y - (length / 2) * math.sin(angle_rad) - (width / 2) * math.cos(angle_rad)

    # Calculate right centre coordinate
    right_x = x + (width / 2) * math.sin(angle_rad)
    right_y = y - (width / 2) * math.cos(angle_rad)

    return (front_right_x, front_right_y), (rear_right_x, rear_right_y), (right_x, right_y)

def point_to_line_distance(x, y, X, Y):
    min_distance = 1000
    n = len(X)
    left = True

    for i in range(n - 1):
        X_i, Y_i = X[i], Y[i]
        X_next, Y_next = X[i + 1], Y[i + 1]

        # Calculate the direction vector of a line segment
        dx = X_next - X_i
        dy = Y_next - Y_i

        # Calculate the projection of the point (x, y) onto the segment (X_i, Y_i) -> (X_next, Y_next)
        if dx != 0 or dy != 0:
            t = ((x - X_i) * dx + (y - Y_i) * dy) / (dx * dx + dy * dy)
            if t < 0:
                px, py = X_i, Y_i  # The projected point falls before the start of the line segment
            elif t > 1:
                px, py = X_next, Y_next  # The projected point falls after the end of the line segment
            else:
                px = X_i + t * dx
                py = Y_i + t * dy

            # Calculate the distance from the point (x, y) to the projected point (px, py).
            distance = math.sqrt((x - px) ** 2 + (y - py) ** 2)
            if distance < min_distance:
                min_distance = distance
                ABx = X_i - x
                ABy = Y_i - y
                ACx = X_next - x
                ACy = Y_next - y
                left = ABx*ACy - ABy*ACx > 0
    if left:
        return min_distance
    else:
        return -min_distance

def search_time_in_local(t, T):
    for i in range(len(T) -1):
        if t > T[i] and t < T[i+1]:
            return i
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Process and analyze control and planning data')
    parser.add_argument('--path', type=str, help='path for bag files')
    args = parser.parse_args()
    front_edge_to_center = 2.151
    right_edge_to_center = 0.505
    if args.path:
        reader = RecordReader(args.path)
        print("Begin reading the file: ", args.path)
        time_stamps = []
        edge_infos = []
        front_edge_infos = []
        local_time_stamps = []
        localization_ego = []
        use_relative_point = False
        if use_relative_point:
            for msg in reader.read_messages():
                #print msg.timestamp,msg.topic
                if msg.topic == "/apollo/perception/edge":
                    edge_info = perception_obstacle_pb2.PerceptionEdgeInfo()
                    edge_info.ParseFromString(msg.message)
                    time_stamps.append(edge_info.header.timestamp_sec)
                    if edge_info.is_useable == False:
                        edge_infos.append(-10.0)
                        front_edge_infos.append(-10.0)
                    else:
                        edge_x = [p.x for p in edge_info.edge_relative_point]
                        edge_y = [p.y for p in edge_info.edge_relative_point]
                        if len(edge_x) < 2:
                            edge_infos.append(-10.0)
                            front_edge_infos.append(-10.0)
                        else:
                            back_index = -1
                            front_index = -1
                            for i in range(len(edge_x) - 1):
                                if back_index < 0 and edge_y[i] < 0.0 and edge_y[i+1] > 0.0:
                                    back_index = i

                                if front_index < 0 and edge_y[i] < front_edge_to_center and edge_y[i+1] > front_edge_to_center:
                                    front_index = i

                            if back_index < 0:
                                edge_infos.append(-10.0)
                            else:
                                delat_y = edge_y[back_index + 1] - edge_y[back_index]
                                edge_infos.append((edge_y[back_index + 1]/delat_y) * edge_x[back_index] + (-edge_y[back_index]/delat_y) * edge_x[back_index+1] - right_edge_to_center)

                            if front_index < 0:
                                front_edge_infos.append(-10.0)
                            else:
                                delat_y = edge_y[front_index + 1] - edge_y[front_index]
                                front_edge_infos.append((edge_y[front_index + 1]/delat_y) * edge_x[front_index] + (-edge_y[front_index]/delat_y) * edge_x[front_index+1] - right_edge_to_center)
        else:
            for msg in reader.read_messages():
                if msg.topic == "/apollo/localization/pose":
                    localization = localization_pb2.LocalizationEstimate()
                    localization.ParseFromString(msg.message)
                    local_time_stamps.append(localization.header.timestamp_sec)
                    localization_ego.append(({'x': localization.pose.position.x, 'y': localization.pose.position.y, 'heading': localization.pose.heading}))
            reader = RecordReader(args.path)
            for msg in reader.read_messages():
                if msg.topic == "/apollo/perception/edge":
                    edge_info = perception_obstacle_pb2.PerceptionEdgeInfo()
                    edge_info.ParseFromString(msg.message)
                    time_stamps.append(edge_info.header.timestamp_sec)

                    if edge_info.is_useable == False:
                        edge_infos.append(-10.0)
                        front_edge_infos.append(-10.0)
                    else:
                        edge_x = [p.x for p in edge_info.edge_point]
                        edge_y = [p.y for p in edge_info.edge_point]

                        local_t_index = search_time_in_local(edge_info.header.timestamp_sec, local_time_stamps)
                        front_right, rear_right, center_right = calculate_corners(localization_ego[local_t_index]['x'], localization_ego[local_t_index]['y'], localization_ego[local_t_index]['heading'])

                        dis = point_to_line_distance(rear_right[0], rear_right[1], edge_x, edge_y)
                        edge_infos.append(dis)
                        dis = point_to_line_distance(front_right[0], front_right[1], edge_x, edge_y)
                        front_edge_infos.append(dis)
                    
        print("Done reading the file: ", args.path)
        plt.figure(1)
        plt.plot(time_stamps, edge_infos, 'r-', label='edge_info back distance')
        plt.plot(time_stamps, front_edge_infos, 'k*', label='edge_info front distance')
        plt.title('edge_info')
        plt.xlabel('Time Stamp')
        plt.ylabel('Distance')
        plt.grid()
        plt.legend()
        plt.show()
