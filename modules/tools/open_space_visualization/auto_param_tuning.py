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

import argparse
import random

from google.protobuf.internal import decoder
from google.protobuf.internal import encoder

from modules.planning.proto import planner_open_space_config_pb2
import common.proto_utils as proto_utils
import distance_approach_visualizer
import hybrid_a_star_visualizer


random.seed(99999)
rand_num = 1000
original_file_path = "/apollo/modules/planning/conf/planner_open_space_config.pb.txt"
optimal_file_path = "/apollo/modules/planning/conf/optimal_planner_open_space_config_-8_4.pb.txt"
# tunning_object = "coarse_trajectory"
tunning_object = "smooth_trajectory"


def load_open_space_protobuf(filename):
    open_space_params = planner_open_space_config_pb2.PlannerOpenSpaceConfig()
    proto_utils.get_pb_from_text_file(filename, open_space_params)
    return open_space_params


def GetParamsForTunning(tunning_object):
    param_names_and_range = []
    if tunning_object == "coarse_trajectory":
        param_names_and_range.append(
            ("warm_start_config.traj_forward_penalty", 2.0))
        param_names_and_range.append(
            ("warm_start_config.traj_back_penalty", 2.0))
        param_names_and_range.append(
            ("warm_start_config.traj_gear_switch_penalty", 2.0))
        param_names_and_range.append(
            ("warm_start_config.traj_steer_penalty", 3.0))
        param_names_and_range.append(
            ("warm_start_config.traj_steer_change_penalty", 2.0))
    elif tunning_object == "smooth_trajectory":
        param_names_and_range.append(
            ("distance_approach_config.weight_steer", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_a", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_steer_rate", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_a_rate", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_x", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_y", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_phi", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_v", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_steer_stitching", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_a_stitching", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_first_order_time", 2.0))
        param_names_and_range.append(
            ("distance_approach_config.weight_second_order_time", 2.0))
    return param_names_and_range


def RandSampling(param_names_and_range, origin_open_space_params):
    params_lists = []
    for iter in range(0, rand_num):
        rand_params = planner_open_space_config_pb2.PlannerOpenSpaceConfig()
        rand_params.CopyFrom(origin_open_space_params)
        for param in param_names_and_range:
            exec("rand_params." +
                 str(param[0]) + "=random.uniform(max(rand_params." +
                 str(param[0])
                 + " - " + str(param[1]) + ",0.0)"
                 + " ,rand_params." + str(param[0]) + " + " + str(param[1]) + ")")
        params_lists.append(rand_params)
    return params_lists


def TestingParams(params_lists, tunning_object):
    key_to_evaluations = {}
    for iter in range(0, len(params_lists)):
        evaluation = ParamEvaluation(params_lists[iter], tunning_object)
        key_to_evaluations[iter] = evaluation
    return key_to_evaluations


def ParamEvaluation(params, tunning_object):
    proto_utils.write_pb_to_text_file(params, original_file_path)
    if tunning_object == "coarse_trajectory":
        visualize_flag = False
        success, x_out, y_out, phi_out, v_out, a_out, steer_out, planning_time = hybrid_a_star_visualizer.HybridAStarPlan(
            visualize_flag)
        if not success:
            return float('inf')
        else:
            return planning_time
    elif tunning_object == "smooth_trajectory":
        visualize_flag = False
        success, opt_x_out, opt_y_out, opt_phi_out, opt_v_out, opt_a_out, opt_steer_out, opt_time_out, planning_time = distance_approach_visualizer.SmoothTrajectory(
            visualize_flag)
        if not success:
            return float('inf')
        else:
            return planning_time


def GetOptimalParams(params_lists, key_to_evaluations):
    tmp = []
    for key, value in key_to_evaluations.items():
        tmptuple = (value, key)
        tmp.append(tmptuple)

    tmp = sorted(tmp)
    optimal_params = params_lists[tmp[0][1]]
    optimal_evaluation = tmp[0][0]
    return optimal_params, optimal_evaluation


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--InputConfig", help="original conf address to be tuned", type=str, default=original_file_path)
    parser.add_argument("--OutputConfig", help="tuned conf address",
                        type=str, default=optimal_file_path)
    parser.add_argument("--TunningObject",
                        help="algorithm to be tuned", type=str, default=tunning_object)
    args = parser.parse_args()
    original_file_path = args.InputConfig
    optimal_file_path = args.OutputConfig
    tunning_object = args.TunningObject
    param_names_and_range = GetParamsForTunning(tunning_object)
    origin_open_space_params = load_open_space_protobuf(original_file_path)
    params_lists = RandSampling(
        param_names_and_range, origin_open_space_params)
    key_to_evaluations = TestingParams(params_lists, tunning_object)
    optimal_params, optimal_evaluation = GetOptimalParams(
        params_lists, key_to_evaluations)
    origin_evaluation = ParamEvaluation(
        origin_open_space_params, tunning_object)
    print("optimal_evaluation is " + str(optimal_evaluation))
    print("origin_evaluation is " + str(origin_evaluation))
    improvement_percentage = (
        origin_evaluation - optimal_evaluation) / origin_evaluation
    print("improvement_percentage is " + str(improvement_percentage))
    proto_utils.write_pb_to_text_file(optimal_params, optimal_file_path)
    proto_utils.write_pb_to_text_file(
        origin_open_space_params, original_file_path)
