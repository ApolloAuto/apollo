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
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_slack_osqp_interface.h"

#include <algorithm>
#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

DualVariableWarmStartSlackOSQPInterface::
    DualVariableWarmStartSlackOSQPInterface(
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
  s_start_index_ = n_start_index_ + 4 * obstacles_num_ * (horizon_ + 1);

  l_warm_up_ = Eigen::MatrixXd::Zero(obstacles_edges_sum_, horizon_ + 1);
  n_warm_up_ = Eigen::MatrixXd::Zero(4 * obstacles_num_, horizon_ + 1);
  slacks_ = Eigen::MatrixXd::Zero(obstacles_num_, horizon_ + 1);

  // get_nlp_info
  lambda_horizon_ = obstacles_edges_sum_ * (horizon_ + 1);
  miu_horizon_ = obstacles_num_ * 4 * (horizon_ + 1);
  slack_horizon_ = obstacles_num_ * (horizon_ + 1);

  // number of variables
  num_of_variables_ = lambda_horizon_ + miu_horizon_ + slack_horizon_;
  // number of constraints
  num_of_constraints_ = 3 * obstacles_num_ * (horizon_ + 1) + num_of_variables_;

  min_safety_distance_ =
      planner_open_space_config.dual_variable_warm_start_config()
          .min_safety_distance();
  check_mode_ =
      planner_open_space_config.dual_variable_warm_start_config().debug_osqp();
  beta_ = planner_open_space_config.dual_variable_warm_start_config().beta();
  osqp_config_ =
      planner_open_space_config.dual_variable_warm_start_config().osqp_config();
}

void DualVariableWarmStartSlackOSQPInterface::printMatrix(
    const int r, const int c, const std::vector<c_float>& P_data,
    const std::vector<c_int>& P_indices, const std::vector<c_int>& P_indptr) {
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

void DualVariableWarmStartSlackOSQPInterface::assembleA(
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

bool DualVariableWarmStartSlackOSQPInterface::optimize() {
  // int kNumParam = num_of_variables_;
  // int kNumConst = num_of_constraints_;

  bool succ = true;
  // assemble P, quadratic term in objective
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  assembleP(&P_data, &P_indices, &P_indptr);
  if (check_mode_) {
    AINFO << "print P_data in whole: ";
    printMatrix(num_of_variables_, num_of_variables_, P_data, P_indices,
                P_indptr);
  }
  // assemble q, linear term in objective, \sum{beta * slacks}
  c_float q[num_of_variables_];  // NOLINT
  for (int i = 0; i < num_of_variables_; ++i) {
    q[i] = 0.0;
    if (i >= s_start_index_) {
      q[i] = beta_;
    }
  }

  // assemble A, linear term in constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  assembleConstraint(&A_data, &A_indices, &A_indptr);
  if (check_mode_) {
    AINFO << "print A_data in whole: ";
    printMatrix(num_of_constraints_, num_of_variables_, A_data, A_indices,
                A_indptr);
    assembleA(num_of_constraints_, num_of_variables_, A_data, A_indices,
              A_indptr);
  }

  // assemble lb & ub, slack_variable <= 0
  c_float lb[num_of_constraints_];  // NOLINT
  c_float ub[num_of_constraints_];  // NOLINT
  int slack_indx = num_of_constraints_ - slack_horizon_;
  for (int i = 0; i < num_of_constraints_; ++i) {
    lb[i] = 0.0;
    if (i >= slack_indx) {
      lb[i] = -2e19;
    }

    if (i < 2 * obstacles_num_ * (horizon_ + 1)) {
      ub[i] = 0.0;
    } else if (i < slack_indx) {
      ub[i] = 2e19;
    } else {
      ub[i] = 0.0;
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
  data->n = num_of_variables_;
  data->m = num_of_constraints_;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lb;
  data->u = ub;

  // Workspace
  OSQPWorkspace* work = osqp_setup(data, settings);

  // Solve Problem
  osqp_solve(work);

  // check state
  if (work->info->status_val != 1 && work->info->status_val != 2) {
    AWARN << "OSQP dual warm up unsuccess, "
          << "return status: " << work->info->status;
    succ = false;
  }

  // transfer to make lambda's norm under 1
  std::vector<double> lambda_norm;
  int l_index = l_start_index_;
  for (int i = 0; i < horizon_ + 1; ++i) {
    int edges_counter = 0;
    for (int j = 0; j < obstacles_num_; ++j) {
      int current_edges_num = obstacles_edges_num_(j, 0);
      Eigen::MatrixXd Aj =
          obstacles_A_.block(edges_counter, 0, current_edges_num, 2);

      double tmp1 = 0;
      double tmp2 = 0;
      for (int k = 0; k < current_edges_num; ++k) {
        tmp1 += Aj(k, 0) * work->solution->x[l_index + k];
        tmp2 += Aj(k, 1) * work->solution->x[l_index + k];
      }

      // norm(A * lambda)
      double tmp_norm = tmp1 * tmp1 + tmp2 * tmp2;
      if (tmp_norm >= 1e-5) {
        lambda_norm.push_back(0.75 / std::sqrt(tmp_norm));
      } else {
        lambda_norm.push_back(0.0);
      }

      edges_counter += current_edges_num;
      l_index += current_edges_num;
    }
  }

  // extract primal results
  int variable_index = 0;
  // 1. lagrange constraint l, [0, obstacles_edges_sum_ - 1] * [0,
  // horizon_l
  for (int i = 0; i < horizon_ + 1; ++i) {
    int r_index = 0;
    for (int j = 0; j < obstacles_num_; ++j) {
      for (int k = 0; k < obstacles_edges_num_(j, 0); ++k) {
        l_warm_up_(r_index, i) = lambda_norm[i * obstacles_num_ + j] *
                                 work->solution->x[variable_index];
        ++variable_index;
        ++r_index;
      }
    }
  }

  // 2. lagrange constraint n, [0, 4*obstacles_num-1] * [0, horizon_]
  for (int i = 0; i < horizon_ + 1; ++i) {
    int r_index = 0;
    for (int j = 0; j < obstacles_num_; ++j) {
      for (int k = 0; k < 4; ++k) {
        n_warm_up_(r_index, i) = lambda_norm[i * obstacles_num_ + j] *
                                 work->solution->x[variable_index];
        ++r_index;
        ++variable_index;
      }
    }
  }

  // 3. slack variables
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      slacks_(j, i) = lambda_norm[i * obstacles_num_ + j] *
                      work->solution->x[variable_index];
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

void DualVariableWarmStartSlackOSQPInterface::checkSolution(
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up) {
  // TODO(Runxin): extend
}

void DualVariableWarmStartSlackOSQPInterface::assembleP(
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
    // current assumption: stationary obstacles
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

  CHECK_EQ(P_indptr->size(), lambda_horizon_);
  for (int i = lambda_horizon_; i < num_of_variables_ + 1; ++i) {
    P_indptr->emplace_back(first_row_location);
  }

  CHECK_EQ(P_data->size(), P_indices->size());
  CHECK_EQ(P_indptr->size(), num_of_variables_ + 1);
}

void DualVariableWarmStartSlackOSQPInterface::assembleConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr) {
  /*
   * The constraint matrix is as the form,
   *  |R' * A',   G', 0|, #: 2 * obstacles_num_ * (horizon_ + 1)
   *  |A * t - b, -g, I|, #: obstacles_num_ * (horizon_ + 1)
   *  |I,          0, 0|, #: num_of_lambda
   *  |0,          I, 0|, #: num_of_miu
   *  |0,          0, I|, #: num_of_slack
   */
  int r1_index = 0;
  int r2_index = 2 * obstacles_num_ * (horizon_ + 1);
  int r3_index = 3 * obstacles_num_ * (horizon_ + 1);
  int r4_index = 3 * obstacles_num_ * (horizon_ + 1) + lambda_horizon_;
  int r5_index = r4_index + miu_horizon_;
  int first_row_location = 0;

  // lambda variables
  // lambda_horizon_ = obstacles_edges_sum_ * (horizon_ + 1);
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

  // miu variables, miu_horizon_ = obstacles_num_ * 4 * (horizon_ + 1);
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

  // slack variables
  // slack_horizon_ = obstacles_edges_sum_ * (horizon_ + 1);
  r2_index = 2 * obstacles_num_ * (horizon_ + 1);
  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      A_data->emplace_back(1.0);
      A_indices->emplace_back(r2_index);
      ++r2_index;

      A_data->emplace_back(1.0);
      A_indices->emplace_back(r5_index);
      ++r5_index;

      A_indptr->emplace_back(first_row_location);
      first_row_location += 2;
    }
  }

  A_indptr->emplace_back(first_row_location);

  CHECK_EQ(A_data->size(), A_indices->size());
  CHECK_EQ(A_indptr->size(), num_of_variables_ + 1);
}

void DualVariableWarmStartSlackOSQPInterface::get_optimization_results(
    Eigen::MatrixXd* l_warm_up, Eigen::MatrixXd* n_warm_up,
    Eigen::MatrixXd* s_warm_up) const {
  *l_warm_up = l_warm_up_;
  *n_warm_up = n_warm_up_;
  *s_warm_up = slacks_;
  // debug mode check slack values
  double max_s = slacks_(0, 0);
  double min_s = slacks_(0, 0);

  for (int i = 0; i < horizon_ + 1; ++i) {
    for (int j = 0; j < obstacles_num_; ++j) {
      max_s = std::max(max_s, slacks_(j, i));
      min_s = std::min(min_s, slacks_(j, i));
    }
  }

  ADEBUG << "max_s: " << max_s;
  ADEBUG << "min_s: " << min_s;
}
}  // namespace planning
}  // namespace apollo
