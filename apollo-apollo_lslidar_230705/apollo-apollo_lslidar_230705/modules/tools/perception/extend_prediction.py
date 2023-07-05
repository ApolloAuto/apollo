#!/usr/bin/env python3

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
print received prediction message
"""
import argparse
import math
import sys

import numpy

import modules.tools.common.proto_utils as proto_utils
from modules.common_msgs.prediction_msgs.prediction_obstacle_pb2 import PredictionObstacles


def distance(p1, p2):
    """distance between two trajectory points"""
    return math.sqrt((p1.y - p2.y)**2 + (p1.x - p2.x)**2)


def get_trajectory_length(trajectory):
    """get_trajectory_length"""
    total = 0.0
    for i in range(1, len(trajectory.trajectory_point)):
        total += distance(trajectory.trajectory_point[i - 1].path_point,
                          trajectory.trajectory_point[i].path_point)
    return total


def extend_prediction(prediction, min_length, min_time):
    """extend prediction"""
    for obstacle in prediction.prediction_obstacle:
        i = 0
        for trajectory in obstacle.trajectory:
            points = trajectory.trajectory_point
            point_num = len(points)
            trajectory_length = get_trajectory_length(trajectory)
            sys.stderr.write("obstacle_id :%s trajectory_id: %s length: %s\n" % (
                obstacle.perception_obstacle.id, i, trajectory_length))
            i += 1
            if trajectory_length < min_length:
                second_last = points[point_num - 2]
                last_point = points[point_num - 1]
                x_diff = last_point.path_point.x - second_last.path_point.x
                y_diff = last_point.path_point.y - second_last.path_point.y
                t_diff = last_point.path_point.theta - second_last.path_point.theta
                delta_diff = math.sqrt(x_diff ** 2 + y_diff ** 2)
                cur_len = trajectory_length
                while cur_len < min_length and abs(cur_len) > 0.000001:
                    last_point.path_point.x += x_diff
                    last_point.path_point.y += y_diff
                    last_point.path_point.theta += t_diff
                    p = points.add()
                    p.CopyFrom(last_point)
                    last_point = p
                    cur_len += delta_diff
    return prediction


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="extend prediction trajectory")
    parser.add_argument("prediction", action="store",
                        type=str, help="set the prediction file")
    parser.add_argument("-p", "--period", action="store", type=float, default=10.0,
                        help="set the prediction period")
    parser.add_argument("-d", "--distance", action="store", type=float, default=70.0,
                        help="set the prediction distance")
    args = parser.parse_args()
    prediction_data = proto_utils.get_pb_from_file(
        args.prediction, PredictionObstacles())
    extended_prediction = extend_prediction(
        prediction_data, args.distance, args.period)
    print(extended_prediction)
