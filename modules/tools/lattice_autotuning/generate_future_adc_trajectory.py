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

import os
import sys
import argparse
import rosbag
import genpy
import glob
import math
from std_msgs.msg import String
import numpy as np

import common.proto_utils as proto_utils
from modules.localization.proto.localization_pb2 import LocalizationEstimate

import logging
logging.basicConfig(level=logging.DEBUG)

def generate_future_pose_list(rosbag_in, period):
    # iteration to generate total trajectory
    total_adc_trajectory = []
    with rosbag.Bag(rosbag_in, 'r') as bag_in:
        for topic, localization, t in bag_in.read_messages():
            if topic == '/apollo/localization/pose':
                total_adc_trajectory.append((t.to_sec(), localization.pose))
    print "Total # Frames for Localization=" + str(len(total_adc_trajectory))

    # dedup trajectory by position
    ret = []
    remove_index_list = []
    for i in range(len(total_adc_trajectory)-1):
        # pair wise
        curr_pose = total_adc_trajectory[i][1];
        next_pose = total_adc_trajectory[i+1][1];
        ds = math.sqrt( \
          (curr_pose.position.x - next_pose.position.x) * \
          (curr_pose.position.x - next_pose.position.x) + \
          (curr_pose.position.y - next_pose.position.y) * \
          (curr_pose.position.y - next_pose.position.y))
        if ds < 0.001:
            remove_index_list.append(i)
    remove_index_list = set(remove_index_list)
    for i in range(len(total_adc_trajectory)):
        if i in remove_index_list:
            continue
        ret.append(total_adc_trajectory[i])
    return ret

def generate_future_traj(rosbag_in, rosbag_out, future_pose_list):
    # Write to bag
    ptr_start = 0
    ptr_end = 800
    if (len(future_pose_list) < ptr_end):
        return
    with rosbag.Bag(rosbag_in, 'r') as bag_in, \
         rosbag.Bag(rosbag_out, 'w') as bag_out:
        for topic, msg, t in bag_in.read_messages():
            if topic == '/apollo/localization/pose' and \
               ptr_end < len(future_pose_list):
                # Augment localization with future localization
                new_localization = msg
                augment_future_traj(new_localization, ptr_start, ptr_end,
                                    future_pose_list)
                bag_out.write(topic, new_localization, t)
                print "Write to bag start =", ptr_start, "end =", ptr_end
                ptr_start += 1
                ptr_end += 1
            else:
                bag_out.write(topic, msg, t)

def augment_future_traj(new_localization, ptr_start, ptr_end, pose_list):
    # Convert a list of pose to traj
    sum_s = 0.0;
    relative_time = 0.0;
    for i in range(ptr_start, ptr_end-1):
        trajectory_point = new_localization.trajectory_point.add()
        pose1 = pose_list[i][1]
        pose2 = pose_list[i+1][1]
        trajectory_point.path_point.x = pose1.position.x
        trajectory_point.path_point.y = pose1.position.y
        trajectory_point.path_point.z = pose1.position.z
        trajectory_point.path_point.theta = pose1.heading
        ds = math.sqrt((pose2.position.y - pose1.position.y) * \
            (pose2.position.y - pose1.position.y) + \
            (pose2.position.x - pose1.position.x) * \
            (pose2.position.x - pose1.position.x))
        trajectory_point.path_point.kappa = \
            (pose2.heading - pose1.heading) / ds;
        trajectory_point.path_point.s = sum_s
        sum_s += ds
        trajectory_point.v = math.sqrt(
          (pose1.linear_velocity.x - pose2.linear_velocity.x) *\
          (pose1.linear_velocity.x - pose2.linear_velocity.x) + \
          (pose1.linear_velocity.y - pose2.linear_velocity.y) *\
          (pose1.linear_velocity.y - pose2.linear_velocity.y))
        trajectory_point.a = math.sqrt(
          (pose1.linear_acceleration.x - pose2.linear_acceleration.x) *\
          (pose1.linear_acceleration.x - pose2.linear_acceleration.x) + \
          (pose1.linear_acceleration.y - pose2.linear_acceleration.y) *\
          (pose1.linear_acceleration.y - pose2.linear_acceleration.y))
        trajectory_point.relative_time = relative_time
        relative_time += pose_list[i+1][0] - pose_list[i][0]
        # not setting dkappa

    # last point
    last_trajectory_point = new_localization.trajectory_point.add()
    last_pose = pose_list[ptr_end-1][1]
    last_trajectory_point.path_point.x = last_pose.position.x
    last_trajectory_point.path_point.y = last_pose.position.y
    last_trajectory_point.path_point.z = last_pose.position.z
    last_trajectory_point.path_point.theta = last_pose.heading
    # kappa is from previous trajectory_point
    last_idx = len(new_localization.trajectory_point) - 1
    last_trajectory_point.path_point.kappa = \
        new_localization.trajectory_point[last_idx].path_point.kappa
    last_trajectory_point.path_point.s = sum_s
    trajectory_point.relative_time = relative_time

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description = "Generate future trajectory based on localization")
    parser.add_argument('path', type = str, help = 'rosbag file or directory')
    parser.add_argument('period', type = float, default = 3.0,
            help = 'duration for future trajectory')
    args = parser.parse_args()
    path = args.path
    period = args.period
    if not os.path.exists(path):
        logging.error("Fail to find path: {}".format(path))
        os._exists(-1)
    if os.path.isdir(path):
        pass
    if os.path.isfile(path):
        bag_name = os.path.splitext(os.path.basename(path))[0]
        path_out = os.path.dirname(path) + '/' + bag_name + \
                  '_with_future_trajectory.bag'
        future_pose_list = generate_future_pose_list(path, period)
        generate_future_traj(path, path_out, future_pose_list)
