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

import os
import numpy as np
import argparse
import google.protobuf
from modules.prediction.proto import offline_features_pb2

distance_threshold = 1.5
"""
future_point {x, y, v_heading, timestamp}
"""

def MergeFutureStatusDicts(dirpath):
    list_of_files = os.listdir(dirpath)
    dict_merged = None

    for file in list_of_files:
        full_path = os.path.join(dirpath, file)
        if file.split('.')[-1] == 'npy' and \
           file.split('.')[-2] == 'future_status':
            dict_curr = np.load(full_path).item()
            if dict_merged is None:
                dict_merged = dict_curr.copy()
            else:
                dict_merged.update(dict_curr)

    return dict_merged


def GetPredictionResultFiles(dirpath):
    list_of_files = os.listdir(dirpath)
    prediction_result_files = []

    for file in list_of_files:
        full_path = os.path.join(dirpath, file)
        if file.split('.')[-1] == 'bin' and \
           file.split('.')[0] == 'prediction_result':
            prediction_result_files.append(full_path)

    return prediction_result_files


def IsCorrectlyPredicted(future_point, prediction_result):
    future_relative_time = future_point[3] - prediction_result.timestamp
    for predicted_traj in prediction_result.trajectory:
        i = 0
        while i + 1 < len(predicted_traj.trajectory_point) and \
              predicted_traj.trajectory_point[i + 1].relative_time < \
              future_relative_time:
            i += 1
        predicted_x = predicted_traj.trajectory_point[i].path_point.x
        predicted_y = predicted_traj.trajectory_point[i].path_point.y
        diff_x = abs(predicted_x - future_point[0])
        diff_y = abs(predicted_y - future_point[1])
        if diff_x < distance_threshold and diff_y < distance_threshold:
            return True
    return False


# return correct_portion, num_trajectory
def CorrectlyPredictePortion(prediction_result, future_status_dict, time_range):
    dict_key = "{}@{:.3f}".format(prediction_result.id,
                                  prediction_result.timestamp)
    if dict_key not in future_status_dict.keys():
        return 0.0, 0.0, 0.0
    obstacle_future_status = future_status_dict[dict_key]
    if obstacle_future_status == None or len(obstacle_future_status) == 0:
        return 0.0, 0.0, 0.0

    portion_correct_predicted = 0.0
    curr_timestamp = obstacle_future_status[0][3]

    total_future_point_count = 0.0
    correct_future_point_count = 0.0
    for future_point in obstacle_future_status:
        if future_point[3] - curr_timestamp > time_range:
            break
        if IsCorrectlyPredicted(future_point, prediction_result):
            correct_future_point_count += 1.0
        total_future_point_count += 1.0
    if total_future_point_count == 0:
        return 0.0, 0.0, 0.0
    portion_correct_predicted = correct_future_point_count / \
                                total_future_point_count

    return portion_correct_predicted, 1.0, len(prediction_result.trajectory)


def Evaluate(dirpath, time_range):
    future_status_dict = MergeFutureStatusDicts(dirpath)
    prediction_result_file_list = GetPredictionResultFiles(dirpath)

    portion_correct_predicted_sum = 0.0
    num_obstacle_sum = 0.0
    num_trajectory_sum = 0.0

    for prediction_result_file_path in prediction_result_file_list:
        list_prediction_result = offline_features_pb2.ListPredictionResult()
        with open(prediction_result_file_path, 'rb') as f:
            list_prediction_result.ParseFromString(f.read())
        for prediction_result in list_prediction_result.prediction_result:
            portion_correct_predicted, num_obstacle, num_trajectory = \
                CorrectlyPredictePortion(prediction_result, future_status_dict, 
                                         time_range)
            portion_correct_predicted_sum += portion_correct_predicted
            num_obstacle_sum += num_obstacle
            num_trajectory_sum += num_trajectory

    print(portion_correct_predicted_sum)
    print(num_obstacle_sum)
    print(num_trajectory_sum)

    return {"portion_correct_predicted_sum": portion_correct_predicted_sum,
            "num_obstacle_sum": num_obstacle_sum,
            "num_trajectory_sum": num_trajectory_sum}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Performance Evaluation')
    parser.add_argument('directory', type=str, help='directory of data')
    parser.add_argument('time_range', type=float, help='time range to evaluate')
    args = parser.parse_args()
    dirpath = args.directory
    time_range = args.time_range
    result_dict = Evaluate(dirpath, time_range)
    np.save(dirpath + "/evaluation_result.npy", result_dict)
