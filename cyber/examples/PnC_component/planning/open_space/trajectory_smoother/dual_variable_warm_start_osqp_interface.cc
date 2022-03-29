/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_osqp_interface.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

DualVariableWarmStartOSQPInterface::DualVariableWarmStartOSQPInterface(
    size_t horizon, double ts, const Eigen::MatrixXd& ego,
    const Eigen::MatrixXi& obstacles_edges_num, const size_t obstacles_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const Eigen::MatrixXd& xWS,
    const PlannerOpenSpaceConfig& planner_open_space_config)
    : ts_(ts),
      ego_(ego),
      obstacles_edges_num_(obstacles_edges_num),
      obstacles_A_(obstacles_A),
      obstacles_b_(obstacles_b),
      xWS_(xWS) {
  ACHECK(horizon < std::numeric_limits<int>::max())
      << "Invalid cast on horizon in open space planner";
  horizon_ = static_cast<int>(horizon);
  ACHECK(obstacles_num < std::numeric_limits<int>::max())
      << "Invalid cast on obstacles_num in open space planner";
  obstacles_num_ = static_cast<int>(obstacles_num);
  w_ev_ = ego_(1, 0) + ego_(3, 0);
  l_ev_ = ego_(0, 0) + ego_(2, 0);
  g_ = {l_ev_ / 2, w_ev_ / 2, l_ev_ / 2, w_ev_ / 2};
  offset_ = (ego_(0, 0) + ego_(2, 0)) / 2 - ego_(2, 0);
  obstacles_edges_sum_ = obstacles_edges_num_.sum();
  l_start_index_ = 0;
  n_start_index_ = l_start_index_ + obstacles_edges_sum_ * (horizon_ + 1);
  l_warm_up_ = Eigen::MatrixXd::Zero(obstacles_edges_sum_, horizon_ + 1);
  n_warm_up_ = Eigen::MatrixXd::Zero(4 * obstacles_num_, horizon_ + 1);

  // get_nlp_info
  lambda_horizon_ = obstacles_edges_sum_ * (horizon_ + 1);

  miu_horizon_ = obstacles_num_ * 4 * (horizon_ + 1);

  // number of variables
  num_of_variables_ = lambda_horizon_ + miu_horizon_;
  // number of constraints
  num_of_constraints_ = 3 * obstacles_num_ * (horizon_ + 1) + num_of_variables_;

  min_safety_distance_ =
      planner_open_space_config.dual_variable_warm_start_config()
          .min_safety_distance();
  check_mode_ =
      planner_open_space_config.dual_variable_warm_start_config().debug_osqp();
  osqp_config_ =
      planner_open_space_config.dual_variable_warm_start_config().osqp_config();
}

void printMatrix(const int r, const int c, const std::vector<c_float>& P_data,
                 const std::vector<c_int>& P_indices,
                 const std::vector<c_int>& P_indptr) {
  Eigen::MatrixXf tmp = Eigen::MatrixXf::Zero(r, c);

  for (size_t i = 0; i < P_indptr.size() - 1; ++i) {
    if (P_indptr[i] < 0 || P_indptr[i] >= static_cast<int>(P_indices.size())) {
      continue;
    }

    for (auto idx = P_indptr[i]; idx < P_indptr[i + 1]; ++idx) {
      int tmp_c = static_cast<int>(i);
      int tmp_r = static_cast<int>(P_indices[idx]);
      tmp(tmp_r, tmp_c) = static_cast<float>(P_data[idx]);
    }
  }

  AINFO << "row number: " << r;
  AINFO << "col number: " << c;
  for (int i = 0; i < r; ++i) {
    AINFO << "row number: " << i;
    AINFO << tmp.row(i);
  }
}

void DualVariableWarmStartOSQPInterface::assembleA(
    const int r, const int c, const std::vector<c_float>& P_data,
    const std::vector<c_int>& P_indices, const std::vector<c_int>& P_indptr) {
  constraint_A_ = Eigen::MatrixXf::Zero(r, c);

  for (size_t i = 0; i < P_indptr.size() - 1; ++i) {
    if (P_indptr[i] < 0 || P_indptr[i] >= static_cast<int>(P_indices.size())) {
      continue;
    }

    for (auto idx = P_indptr[i]; idx < P_indptr[i + 1]; ++idx) {
      int tmp_c = static_cast<int>(i);
      int tmp_r = static_cast<int>(P_indices[idx]);
      constraint_A_(tmp_r, tmp_c) = static_cast<float>(P_data[idx]);
    }
  }
}

bool DualVariableWarmStartOSQPInterface::optimize() {
  int kNumParam = num_of_variables_;
  int kNumConst = num_of_constraints_;

  bool succ = true;
  // assemble P, quadratic term in objective
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  assemble_P(&P_data, &P_indices, &P_indptr);
  if (check_mode_) {
    AINFO << "print P_data in whole: ";
    printMatrix(kNumParam, kNumParam, P_data, P_indices, P_indptr);
  }
  // assemble q, linear term in objective
  c_float q[kNumParam];
  for (int i = 0; i < kNumParam; ++i) {
    q[i] = 0.0;
  }

  // assemble A, linear term in constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  assemble_constraint(&A_data, &A_indices, &A_indptr);
  if (check_mode_) {
    AINFO << "print A_data in whole: ";
    printMatrix(kNumConst, kNumParam, A_data, A_indices, A_indptr);
    assembleA(kNumConst, kNumParam, A_data, A_indices, A_indptr);
  }

  // assemble lb & ub
  c_float lb[kNumConst];
  c_float ub[kNumConst];
  for (int i = 0; i < kNumConst; ++i) {
    lb[i] = 0.0;
    if (i >= 2 * obstacles_num_ * (horizon_ + 1) &&
        i < 3 * obstacles_num_ * (horizon_ + 1)) {
      lb[i] = min_safety_distance_;
    }
    if (i < 2 * obstacles_num_ * (horizon_ + 1)) {
      ub[i] = 0.0;
    } else {
      ub[i] = 2e19;
    }
  }

  // Problem settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = osqp_config_.alpha();  // Change alpha parameter
  settings->eps_abs = osqp_config_.eps_abs();
  settings->eps_rel = osqp_config_.eps_rel();
  settings->max_iter = osqp_config_.max_iter();
  settings->polish = osqp_config_.polish();
  settings->verbose = osqp_config_.osqp_debug_log();

  // Populate data
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = kNumParam;
  data->m = kNumConst;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lb;
  data->u = ub;

  // Workspace
  OSQPWorkspace* work = nullptr;
  // osqp_setup(&work, data, settings);
  work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  // check state
  if (work->info->status_val != 1 && work->info->status_val != 2) {
    AWARN << "OSQP dual warm up unsuccess, "
          << "return status: " << work->info->status;
    succ = false;
  }

  // extract primal results
  int variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_l
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      l_warm_up_(j, i) = work->solution->x[variable_index];
      ++variable_index;
    }
  }

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      n_warm_up_(j, i) = work->solution->x[variable_index];
      ++variable_index;
    }
  }

  succ = succ & (work->info->obj_val <= 1.0);

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return succ;
}

void DualVariableWarmStartOSQPInterface::check_solution(
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up) {
  Eigen::MatrixXf x(num_of_variables_, 1);
  Eigen::MatrixXf g(num_of_constraints_, 1);

  // extract primal results
  int variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_l
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_edges_sum_; ++j) {
      x(variable_index, 0) = static_cast<float>(l_warm_up(j, i));
      ++variable_index;
    }
  }

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < 4 * obstacles_num_; ++j) {
      x(variable_index, 0) = static_cast<float>(n_warm_up(j, i));
      ++variable_index;
    }
  }

  g = constraint_A_ * x;
  int r1_index = 0;
  int r2_index = 2 * obstacles_num_ * (horizon_ + 1);
  int r3_index = 3 * obstacles_num_ * (horizon_ + 1);
  int r4_index = 3 * obstacles_num_ * (horizon_ + 1) + lambda_horizon_;

  for (int idx = r1_index; idx < r2_index; ++idx) {
    if (std::abs(g(idx, 0)) > 1e-6) {
      AERROR << "G' * mu + R' * A * lambda == 0 constraint fails, "
             << "constraint_index: " << idx << ", g: " << g(idx, 0);
    }
  }

  for (int idx = r2_index; idx < r3_index; ++idx) {
    if (g(idx, 0) < min_safety_distance_) {
      AERROR << "-g' * mu + (A * t - b) * lambda) >= d_min constraint fails, "
             << "constraint_index: " << idx << ", g: " << g(idx, 0);
    }
  }

  for (int idx = r3_index; idx < r4_index; ++idx) {
    if (g(idx, 0) < 0) {
      AERROR << "lambda box constraint fails, "
             << "constraint_index: " << idx << ", g: " << g(idx, 0);
    }
  }

  for (int idx = r4_index; idx < num_of_constraints_; ++idx) {
    if (g(idx, 0) < 0) {
      AERROR << "miu box constraint fails, "
             << "constraint_index: " << idx << ", g: " << g(idx, 0);
    }
  }
}

void DualVariableWarmStartOSQPInterface::assemble_P(
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  // the objective function is norm(A' * lambda)
  std::vector<c_float> P_tmp;
  int edges_counter = 0;

  for (int j = 0; j < obstacles_num_; ++j) {
    int current_edges_num = obstacles_edges_num_(j, 0);
    Eigen::MatrixXd Aj;
    Aj = obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
    // Eigen::MatrixXd AAj(current_edges_num, current_edges_num);
    Aj = Aj * Aj.transpose();

    CHECK_EQ(current_edges_num, Aj.cols());
    CHECK_EQ(current_edges_num, Aj.rows());

    for (int c = 0; c < current_edges_num; ++c) {
      for (int r = 0; r < current_edges_num; ++r) {
        P_tmp.emplace_back(Aj(r, c));
      }
    }

    // Update index
    edges_counter += current_edges_num;
  }

  int l_index = l_start_index_;
  int first_row_location = 0;
  // the objective function is norm(A' * lambda)
  for (int i = 0; i < horizon_ + 1; ++i) {
    edges_counter = 0;

    for (auto item : P_tmp) {
      P_data->emplace_back(item);
    }
    // current assume: stationary obstacles
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);

      for (int c = 0; c < current_edges_num; ++c) {
        P_indptr->emplace_back(first_row_location);
        for (int r = 0; r < current_edges_num; ++r) {
          P_indices->emplace_back(r + l_index);
        }
        first_row_location += current_edges_num;
      }

      // Update index
      edges_counter += current_edges_num;
      l_index += current_edges_num;
    }
  }

  CHECK_EQ(P_indptr->size(), static_cast<size_t>(lambda_horizon_));
  for (int i = lambda_horizon_; i < num_of_variables_ + 1; ++i) {
    P_indptr->emplace_back(first_row_location);
  }

  CHECK_EQ(P_data->size(), P_indices->size());
  CHECK_EQ(P_indptr->size(), static_cast<size_t>(num_of_variables_) + 1);
}

void DualVariableWarmStartOSQPInterface::assemble_constraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr) {
  /*
   * The constraint matrix is as the form,
   *  |R' * A',   G'|, #: 2 * obstacles_num_ * (horizon_ + 1)
   *  |A * t - b, -g|, #: obstacles_num_ * (horizon_ + 1)
   *  |I,          0|, #: num_of_lambda
   *  |0,          I|, #: num_of_miu
   */
  int r1_index = 0;
  int r2_index = 2 * obstacles_num_ * (horizon_ + 1);
  int r3_index = 3 * obstacles_num_ * (horizon_ + 1);
  int r4_index = 3 * obstacles_num_ * (horizon_ + 1) + lambda_horizon_;
  int first_row_location = 0;

  // lambda variables
  for (int i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;

    Eigen::MatrixXd R(2, 2);
    R << cos(xWS_(2, i)), sin(xWS_(2, i)), sin(xWS_(2, i)), cos(xWS_(2, i));

    Eigen::MatrixXd t_trans(1, 2);
    t_trans << (xWS_(0, i) + cos(xWS_(2, i)) * offset_),
        (xWS_(1, i) + sin(xWS_(2, i)) * offset_);

    // assume: stationary obstacles
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);
      Eigen::MatrixXd bj =
          obstacles_b_.block(edges_counter, 0, current_edges_num, 1);

      Eigen::MatrixXd r1_block(2, current_edges_num);
      r1_block = R * Aj.transpose();

      Eigen::MatrixXd r2_block(1, current_edges_num);
      r2_block = t_trans * Aj.transpose() - bj.transpose();

      // insert into A matrix, col by col
      for (int k = 0; k < current_edges_num; ++k) {
        A_data->emplace_back(r1_block(0, k));
        A_indices->emplace_back(r1_index);

        A_data->emplace_back(r1_block(1, k));
        A_indices->emplace_back(r1_index + 1);

        A_data->emplace_back(r2_block(0, k));
        A_indices->emplace_back(r2_index);

        A_data->emplace_back(1.0);
        A_indices->emplace_back(r3_index);
        r3_index++;

        A_indptr->emplace_back(first_row_location);
        first_row_location += 4;
      }

      // Update index
      edges_counter += current_edges_num;
      r1_index += 2;
      r2_index += 1;
    }
  }

  // miu variables
  // G: ((1, 0, -1, 0), (0, 1, 0, -1))
  // g: g_
  r1_index = 0;
  r2_index = 2 * obstacles_num_ * (horizon_ + 1);
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      for (int k = 0; k < 4; ++k) {
        // update G
        if (k < 2) {
          A_data->emplace_back(1.0);
        } else {
          A_data->emplace_back(-1.0);
        }
        A_indices->emplace_back(r1_index + k % 2);

        // update g'
        A_data->emplace_back(-g_[k]);
        A_indices->emplace_back(r2_index);

        // update I
        A_data->emplace_back(1.0);
        A_indices->emplace_back(r4_index);
        r4_index++;

        // update col index
        A_indptr->emplace_back(first_row_location);
        first_row_location += 3;
      }

      // update index
      r1_index += 2;
      r2_index += 1;
    }
  }

  A_indptr->emplace_back(first_row_location);

  CHECK_EQ(A_data->size(), A_indices->size());
  CHECK_EQ(A_indptr->size(), static_cast<size_t>(num_of_variables_) + 1);
}

void DualVariableWarmStartOSQPInterface::get_optimization_results(
    Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up) const {
  *l_warm_up = l_warm_up_;
  *n_warm_up = n_warm_up_;
}
}  // namespace planning
}  // namespace apollo
