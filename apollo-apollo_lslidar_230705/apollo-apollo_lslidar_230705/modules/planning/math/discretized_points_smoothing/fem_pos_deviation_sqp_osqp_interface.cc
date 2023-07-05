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

/**
 * @file
 **/

#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.h"

#include <cmath>
#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

bool FemPosDeviationSqpOsqpInterface::Solve() {
  // Sanity Check
  if (ref_points_.empty()) {
    AERROR << "reference points empty, solver early terminates";
    return false;
  }

  if (ref_points_.size() != bounds_around_refs_.size()) {
    AERROR << "ref_points and bounds size not equal, solver early terminates";
    return false;
  }

  if (ref_points_.size() < 3) {
    AERROR << "ref_points size smaller than 3, solver early terminates";
    return false;
  }

  if (ref_points_.size() > std::numeric_limits<int>::max()) {
    AERROR << "ref_points size too large, solver early terminates";
    return false;
  }

  // Calculate optimization states definitions
  num_of_points_ = static_cast<int>(ref_points_.size());
  num_of_pos_variables_ = num_of_points_ * 2;
  num_of_slack_variables_ = num_of_points_ - 2;
  num_of_variables_ = num_of_pos_variables_ + num_of_slack_variables_;

  num_of_variable_constraints_ = num_of_variables_;
  num_of_curvature_constraints_ = num_of_points_ - 2;
  num_of_constraints_ =
      num_of_variable_constraints_ + num_of_curvature_constraints_;

  // Set primal warm start
  std::vector<c_float> primal_warm_start;
  SetPrimalWarmStart(ref_points_, &primal_warm_start);

  // Calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // Calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);

  // Calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(ref_points_, &A_data, &A_indices, &A_indptr,
                            &lower_bounds, &upper_bounds);

  // Load matrices and vectors into OSQPData
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = num_of_variables_;
  data->m = num_of_constraints_;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  // Define osqp solver settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->max_iter = max_iter_;
  settings->time_limit = time_limit_;
  settings->verbose = verbose_;
  settings->scaled_termination = scaled_termination_;
  settings->warm_start = warm_start_;
  settings->polish = true;
  settings->eps_abs = 1e-5;
  settings->eps_rel = 1e-5;
  settings->eps_prim_inf = 1e-5;
  settings->eps_dual_inf = 1e-5;

  // Define osqp workspace
  OSQPWorkspace* work = nullptr;
  // osqp_setup(&work, data, settings);
  work = osqp_setup(data, settings);

  // Initial solution
  bool initial_solve_res = OptimizeWithOsqp(primal_warm_start, &work);

  if (!initial_solve_res) {
    AERROR << "initial iteration solving fails";
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);
    return false;
  }

  // Sequential solution

  int pen_itr = 0;
  double ctol = 0.0;
  double original_slack_penalty = weight_curvature_constraint_slack_var_;
  double last_fvalue = work->info->obj_val;

  while (pen_itr < sqp_pen_max_iter_) {
    int sub_itr = 1;
    bool fconverged = false;

    while (sub_itr < sqp_sub_max_iter_) {
      SetPrimalWarmStart(opt_xy_, &primal_warm_start);
      CalculateOffset(&q);
      CalculateAffineConstraint(opt_xy_, &A_data, &A_indices, &A_indptr,
                                &lower_bounds, &upper_bounds);
      osqp_update_lin_cost(work, q.data());
      osqp_update_A(work, A_data.data(), OSQP_NULL, A_data.size());
      osqp_update_bounds(work, lower_bounds.data(), upper_bounds.data());

      bool iterative_solve_res = OptimizeWithOsqp(primal_warm_start, &work);
      if (!iterative_solve_res) {
        AERROR << "iteration at " << sub_itr
               << ", solving fails with max sub iter " << sqp_sub_max_iter_;
        weight_curvature_constraint_slack_var_ = original_slack_penalty;
        osqp_cleanup(work);
        c_free(data->A);
        c_free(data->P);
        c_free(data);
        c_free(settings);
        return false;
      }

      double cur_fvalue = work->info->obj_val;
      double ftol = std::abs((last_fvalue - cur_fvalue) / last_fvalue);

      if (ftol < sqp_ftol_) {
        ADEBUG << "merit function value converges at sub itr num " << sub_itr;
        ADEBUG << "merit function value converges to " << cur_fvalue
               << ", with ftol " << ftol << ", under max_ftol " << sqp_ftol_;
        fconverged = true;
        break;
      }

      last_fvalue = cur_fvalue;
      ++sub_itr;
    }

    if (!fconverged) {
      AERROR << "Max number of iteration reached";
      weight_curvature_constraint_slack_var_ = original_slack_penalty;
      osqp_cleanup(work);
      c_free(data->A);
      c_free(data->P);
      c_free(data);
      c_free(settings);
      return false;
    }

    ctol = CalculateConstraintViolation(opt_xy_);

    ADEBUG << "ctol is " << ctol << ", at pen itr " << pen_itr;

    if (ctol < sqp_ctol_) {
      ADEBUG << "constraint satisfied at pen itr num " << pen_itr;
      ADEBUG << "constraint voilation value drops to " << ctol
             << ", under max_ctol " << sqp_ctol_;
      weight_curvature_constraint_slack_var_ = original_slack_penalty;
      osqp_cleanup(work);
      c_free(data->A);
      c_free(data->P);
      c_free(data);
      c_free(settings);
      return true;
    }

    weight_curvature_constraint_slack_var_ *= 10;
    ++pen_itr;
  }

  ADEBUG << "constraint not satisfied with total itr num " << pen_itr;
  ADEBUG << "constraint voilation value drops to " << ctol
         << ", higher than max_ctol " << sqp_ctol_;
  weight_curvature_constraint_slack_var_ = original_slack_penalty;
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);
  return true;
}

void FemPosDeviationSqpOsqpInterface::CalculateKernel(
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  CHECK_GT(num_of_points_, 2);

  // Three quadratic penalties are involved:
  // 1. Penalty x on distance between middle point and point by finite element
  // estimate;
  // 2. Penalty y on path length;
  // 3. Penalty z on difference between points and reference points

  // General formulation of P matrix is as below(with 6 points as an example):
  // I is a two by two identity matrix, X, Y, Z represents x * I, y * I, z * I
  // 0 is a two by two zero matrix
  // |X+Y+Z, -2X-Y,   X,       0,       0,       0,       ...|
  // |0,     5X+2Y+Z, -4X-Y,   X,       0,       0,       ...|
  // |0,     0,       6X+2Y+Z, -4X-Y,   X,       0,       ...|
  // |0,     0,       0,       6X+2Y+Z, -4X-Y,   X        ...|
  // |0,     0,       0,       0,       5X+2Y+Z, -2X-Y    ...|
  // |0,     0,       0,       0,       0,       X+Y+Z    ...|
  // |0,     0,       0,       0,       0,       0,       0,       ...|
  // |0,     0,       0,       0,       0,       0,       0, 0,       ...|
  // |0,     0,       0,       0,       0,       0,       0, 0, 0,       ...|
  // |0,     0,       0,       0,       0,       0,       0, 0, 0, 0,       ...|
  // Only upper triangle needs to be filled
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_of_variables_);
  int col_num = 0;

  for (int col = 0; col < 2; ++col) {
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  for (int col = 2; col < 4; ++col) {
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  int second_point_from_last_index = num_of_points_ - 2;
  for (int point_index = 2; point_index < second_point_from_last_index;
       ++point_index) {
    int col_index = point_index * 2;
    for (int col = 0; col < 2; ++col) {
      col_index += col;
      columns[col_index].emplace_back(col_index - 4, weight_fem_pos_deviation_);
      columns[col_index].emplace_back(
          col_index - 2,
          -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
      columns[col_index].emplace_back(
          col_index, 6.0 * weight_fem_pos_deviation_ +
                         2.0 * weight_path_length_ + weight_ref_deviation_);
      ++col_num;
    }
  }

  int second_point_col_from_last_col = num_of_pos_variables_ - 4;
  int last_point_col_from_last_col = num_of_pos_variables_ - 2;
  for (int col = second_point_col_from_last_col;
       col < last_point_col_from_last_col; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);
    columns[col].emplace_back(
        col - 2, -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  for (int col = last_point_col_from_last_col; col < num_of_pos_variables_;
       ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  CHECK_EQ(col_num, num_of_pos_variables_);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables_; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      // Rescale by 2.0 as the quadratic term in osqp default qp problem setup
      // is set as (1/2) * x' * P * x
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void FemPosDeviationSqpOsqpInterface::CalculateOffset(std::vector<c_float>* q) {
  q->resize(num_of_variables_);
  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    (*q)[2 * i] = -2.0 * weight_ref_deviation_ * ref_point_xy.first;
    (*q)[2 * i + 1] = -2.0 * weight_ref_deviation_ * ref_point_xy.second;
  }
  for (int i = 0; i < num_of_slack_variables_; ++i) {
    (*q)[num_of_pos_variables_ + i] = weight_curvature_constraint_slack_var_;
  }
}

std::vector<double>
FemPosDeviationSqpOsqpInterface::CalculateLinearizedFemPosParams(
    const std::vector<std::pair<double, double>>& points, const size_t index) {
  CHECK_GT(index, 0U);
  CHECK_LT(index, points.size() - 1);

  double x_f = points[index - 1].first;
  double x_m = points[index].first;
  double x_l = points[index + 1].first;
  double y_f = points[index - 1].second;
  double y_m = points[index].second;
  double y_l = points[index + 1].second;

  double linear_term_x_f = 2.0 * x_f - 4.0 * x_m + 2.0 * x_l;
  double linear_term_x_m = 8.0 * x_m - 4.0 * x_f - 4.0 * x_l;
  double linear_term_x_l = 2.0 * x_l - 4.0 * x_m + 2.0 * x_f;
  double linear_term_y_f = 2.0 * y_f - 4.0 * y_m + 2.0 * y_l;
  double linear_term_y_m = 8.0 * y_m - 4.0 * y_f - 4.0 * y_l;
  double linear_term_y_l = 2.0 * y_l - 4.0 * y_m + 2.0 * y_f;

  double linear_approx = (-2.0 * x_m + x_f + x_l) * (-2.0 * x_m + x_f + x_l) +
                         (-2.0 * y_m + y_f + y_l) * (-2.0 * y_m + y_f + y_l) +
                         -x_f * linear_term_x_f + -x_m * linear_term_x_m +
                         -x_l * linear_term_x_l + -y_f * linear_term_y_f +
                         -y_m * linear_term_y_m + -y_l * linear_term_y_l;

  return {linear_term_x_f, linear_term_y_f, linear_term_x_m, linear_term_y_m,
          linear_term_x_l, linear_term_y_l, linear_approx};
}

void FemPosDeviationSqpOsqpInterface::CalculateAffineConstraint(
    const std::vector<std::pair<double, double>>& points,
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  const double scale_factor = 1;

  std::vector<std::vector<double>> lin_cache;
  for (int i = 1; i < num_of_points_ - 1; ++i) {
    lin_cache.push_back(CalculateLinearizedFemPosParams(points, i));
  }

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(num_of_variables_);

  for (int i = 0; i < num_of_variables_; ++i) {
    columns[i].emplace_back(i, 1.0);
  }

  for (int i = num_of_pos_variables_; i < num_of_variables_; ++i) {
    columns[i].emplace_back(i + num_of_slack_variables_, -1.0 * scale_factor);
  }

  for (int i = 2; i < num_of_points_; ++i) {
    int index = 2 * i;
    columns[index].emplace_back(i - 2 + num_of_variables_,
                                lin_cache[i - 2][4] * scale_factor);
    columns[index + 1].emplace_back(i - 2 + num_of_variables_,
                                    lin_cache[i - 2][5] * scale_factor);
  }

  for (int i = 1; i < num_of_points_ - 1; ++i) {
    int index = 2 * i;
    columns[index].emplace_back(i - 1 + num_of_variables_,
                                lin_cache[i - 1][2] * scale_factor);
    columns[index + 1].emplace_back(i - 1 + num_of_variables_,
                                    lin_cache[i - 1][3] * scale_factor);
  }

  for (int i = 0; i < num_of_points_ - 2; ++i) {
    int index = 2 * i;
    columns[index].emplace_back(i + num_of_variables_,
                                lin_cache[i][0] * scale_factor);
    columns[index + 1].emplace_back(i + num_of_variables_,
                                    lin_cache[i][1] * scale_factor);
  }

  int ind_a = 0;
  for (int i = 0; i < num_of_variables_; ++i) {
    A_indptr->push_back(ind_a);
    for (const auto& row_data_pair : columns[i]) {
      A_data->push_back(row_data_pair.second);
      A_indices->push_back(row_data_pair.first);
      ++ind_a;
    }
  }
  A_indptr->push_back(ind_a);

  lower_bounds->resize(num_of_constraints_);
  upper_bounds->resize(num_of_constraints_);

  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    (*upper_bounds)[i * 2] = ref_point_xy.first + bounds_around_refs_[i];
    (*upper_bounds)[i * 2 + 1] = ref_point_xy.second + bounds_around_refs_[i];
    (*lower_bounds)[i * 2] = ref_point_xy.first - bounds_around_refs_[i];
    (*lower_bounds)[i * 2 + 1] = ref_point_xy.second - bounds_around_refs_[i];
  }

  for (int i = 0; i < num_of_slack_variables_; ++i) {
    (*upper_bounds)[num_of_pos_variables_ + i] = 1e20;
    (*lower_bounds)[num_of_pos_variables_ + i] = 0.0;
  }

  double interval_sqr = average_interval_length_ * average_interval_length_;
  double curvature_constraint_sqr = (interval_sqr * curvature_constraint_) *
                                    (interval_sqr * curvature_constraint_);
  for (int i = 0; i < num_of_curvature_constraints_; ++i) {
    (*upper_bounds)[num_of_variable_constraints_ + i] =
        (curvature_constraint_sqr - lin_cache[i][6]) * scale_factor;
    (*lower_bounds)[num_of_variable_constraints_ + i] = -1e20;
  }
}

void FemPosDeviationSqpOsqpInterface::SetPrimalWarmStart(
    const std::vector<std::pair<double, double>>& points,
    std::vector<c_float>* primal_warm_start) {
  CHECK_EQ(points.size(), static_cast<size_t>(num_of_points_));
  CHECK_GT(points.size(), 1U);

  // Set states
  primal_warm_start->resize(num_of_variables_);
  for (int i = 0; i < num_of_points_; ++i) {
    (*primal_warm_start)[2 * i] = points[i].first;
    (*primal_warm_start)[2 * i + 1] = points[i].second;
  }

  slack_.resize(num_of_slack_variables_);
  for (int i = 0; i < num_of_slack_variables_; ++i) {
    (*primal_warm_start)[num_of_pos_variables_ + i] = slack_[i];
  }

  // Calculate average interval length for curvature constraints
  double total_length = 0.0;
  auto pre_point = points.front();
  for (int i = 1; i < num_of_points_; ++i) {
    const auto& cur_point = points[i];
    total_length += std::sqrt((pre_point.first - cur_point.first) *
                                  (pre_point.first - cur_point.first) +
                              (pre_point.second - cur_point.second) *
                                  (pre_point.second - cur_point.second));
    pre_point = cur_point;
  }
  average_interval_length_ = total_length / (num_of_points_ - 1);
}

bool FemPosDeviationSqpOsqpInterface::OptimizeWithOsqp(
    const std::vector<c_float>& primal_warm_start, OSQPWorkspace** work) {
  osqp_warm_start_x(*work, primal_warm_start.data());

  // Solve Problem
  osqp_solve(*work);

  auto status = (*work)->info->status_val;

  if (status < 0) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  if (status != 1 && status != 2) {
    AERROR << "failed optimization status:\t" << (*work)->info->status;
    return false;
  }

  // Extract primal results
  opt_xy_.resize(num_of_points_);
  slack_.resize(num_of_slack_variables_);
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    opt_xy_.at(i) = std::make_pair((*work)->solution->x[index],
                                   (*work)->solution->x[index + 1]);
  }

  for (int i = 0; i < num_of_slack_variables_; ++i) {
    slack_.at(i) = (*work)->solution->x[num_of_pos_variables_ + i];
  }

  return true;
}

double FemPosDeviationSqpOsqpInterface::CalculateConstraintViolation(
    const std::vector<std::pair<double, double>>& points) {
  CHECK_GT(points.size(), 2U);

  double total_length = 0.0;
  auto pre_point = points.front();
  for (int i = 1; i < num_of_points_; ++i) {
    const auto& cur_point = points[i];
    total_length += std::sqrt((pre_point.first - cur_point.first) *
                                  (pre_point.first - cur_point.first) +
                              (pre_point.second - cur_point.second) *
                                  (pre_point.second - cur_point.second));
    pre_point = cur_point;
  }
  double average_interval_length = total_length / (num_of_points_ - 1);
  double interval_sqr = average_interval_length * average_interval_length;
  double curvature_constraint_sqr = (interval_sqr * curvature_constraint_) *
                                    (interval_sqr * curvature_constraint_);

  double max_cviolation = std::numeric_limits<double>::min();
  for (size_t i = 1; i < points.size() - 1; ++i) {
    double x_f = points[i - 1].first;
    double x_m = points[i].first;
    double x_l = points[i + 1].first;
    double y_f = points[i - 1].second;
    double y_m = points[i].second;
    double y_l = points[i + 1].second;
    double cviolation = curvature_constraint_sqr -
                        (-2.0 * x_m + x_f + x_l) * (-2.0 * x_m + x_f + x_l) +
                        (-2.0 * y_m + y_f + y_l) * (-2.0 * y_m + y_f + y_l);
    max_cviolation = max_cviolation < cviolation ? cviolation : max_cviolation;
  }
  return max_cviolation;
}

}  // namespace planning
}  // namespace apollo
