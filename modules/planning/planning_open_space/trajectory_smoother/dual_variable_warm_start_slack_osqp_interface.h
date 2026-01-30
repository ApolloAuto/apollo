/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once

#include <limits>
#include <vector>

#include "Eigen/Dense"
#include "osqp/osqp.h"

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/planning/planning_open_space/proto/planner_open_space_config.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

class DualVariableWarmStartSlackOSQPInterface {
public:
    DualVariableWarmStartSlackOSQPInterface(
            size_t horizon,
            double ts,
            const Eigen::MatrixXd& ego,
            const Eigen::MatrixXi& obstacles_edges_num,
            const size_t obstacles_num,
            const Eigen::MatrixXd& obstacles_A,
            const Eigen::MatrixXd& obstacles_b,
            const Eigen::MatrixXd& xWS,
            const PlannerOpenSpaceConfig& planner_open_space_config);

    virtual ~DualVariableWarmStartSlackOSQPInterface() = default;

    void get_optimization_results(Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* s_warm_up)
            const;

    bool optimize();

    void assembleP(std::vector<c_float>* P_data, std::vector<c_int>* P_indices, std::vector<c_int>* P_indptr);

    void assembleConstraint(std::vector<c_float>* A_data, std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr);

    void assembleA(
            const int r,
            const int c,
            const std::vector<c_float>& P_data,
            const std::vector<c_int>& P_indices,
            const std::vector<c_int>& P_indptr);

    void checkSolution(const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up);

    void printMatrix(
            const int r,
            const int c,
            const std::vector<c_float>& P_data,
            const std::vector<c_int>& P_indices,
            const std::vector<c_int>& P_indptr);

private:
    OSQPConfig osqp_config_;

    int num_of_variables_;
    int num_of_constraints_;
    int horizon_;
    double ts_;
    Eigen::MatrixXd ego_;
    int lambda_horizon_ = 0;
    int miu_horizon_ = 0;
    int slack_horizon_ = 0;
    double beta_ = 0.0;

    Eigen::MatrixXd l_warm_up_;
    Eigen::MatrixXd n_warm_up_;
    Eigen::MatrixXd slacks_;
    double wheelbase_;

    double w_ev_;
    double l_ev_;
    std::vector<double> g_;
    double offset_;
    Eigen::MatrixXi obstacles_edges_num_;
    int obstacles_num_;
    int obstacles_edges_sum_;

    double min_safety_distance_;

    // lagrangian l start index
    int l_start_index_ = 0;

    // lagrangian n start index
    int n_start_index_ = 0;

    // slack s start index
    int s_start_index_ = 0;

    // obstacles_A
    Eigen::MatrixXd obstacles_A_;

    // obstacles_b
    Eigen::MatrixXd obstacles_b_;

    // states of warm up stage
    Eigen::MatrixXd xWS_;

    // constraint A matrix in eigen format
    Eigen::MatrixXf constraint_A_;

    bool check_mode_;

    // park generic
public:
    DualVariableWarmStartSlackOSQPInterface(
            size_t horizon,
            double ts,
            const Eigen::MatrixXd& ego,
            const Eigen::MatrixXi& obstacles_edges_num,
            const size_t obstacles_num,
            const Eigen::MatrixXd& obstacles_A,
            const Eigen::MatrixXd& obstacles_b,
            const Eigen::MatrixXd& xWS,
            const DualVariableWarmStartConfig& dual_variable_warm_start_config);
};

}  // namespace planning
}  // namespace apollo
