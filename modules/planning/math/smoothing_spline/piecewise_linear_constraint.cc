/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @brief: Implementation of PiecewiseLinearConstraint class
 **/

#include "modules/planning/math/smoothing_spline/piecewise_linear_constraint.h"

#include <limits>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

namespace {

Eigen::MatrixXd MergeMaxtrices(const std::vector<Eigen::MatrixXd> matrices) {
  int32_t d = 0;
  for (const auto& mat : matrices) {
    d += mat.rows();
  }
  int32_t col = matrices.front().cols();
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(d, col);
  int32_t index = 0;
  for (const auto& mat : matrices) {
    res.block(index, 0, mat.rows(), mat.cols()) = mat;
    index += mat.rows();
  }
  return res;
}
}  // namespace

PiecewiseLinearConstraint::PiecewiseLinearConstraint(const uint32_t dimension,
                                                     const double unit_segment)
    : dimension_(dimension), unit_segment_(unit_segment) {}

Eigen::MatrixXd PiecewiseLinearConstraint::inequality_constraint_matrix()
    const {
  return MergeMaxtrices(inequality_matrices_);
}

Eigen::MatrixXd PiecewiseLinearConstraint::inequality_constraint_boundary()
    const {
  return MergeMaxtrices(inequality_boundaries_);
}

Eigen::MatrixXd PiecewiseLinearConstraint::equality_constraint_matrix() const {
  return MergeMaxtrices(equality_matrices_);
}

Eigen::MatrixXd PiecewiseLinearConstraint::equality_constraint_boundary()
    const {
  return MergeMaxtrices(equality_boundaries_);
}

bool PiecewiseLinearConstraint::AddBoundary(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  if (index_list.size() != lower_bound.size() ||
      index_list.size() != upper_bound.size()) {
    AERROR << "The sizes of index list, lower_bound, upper_bound are not "
              "identical.";
    return false;
  }
  Eigen::MatrixXd inequality_matrix =
      Eigen::MatrixXd::Zero(2 * index_list.size(), dimension_);
  Eigen::MatrixXd inequality_boundary =
      Eigen::MatrixXd::Zero(2 * index_list.size(), 1);

  for (uint32_t i = 0; i < index_list.size(); ++i) {
    uint32_t index = index_list[i];
    inequality_matrix(2 * i, index) = -1.0;
    inequality_boundary(2 * i, 0) = -upper_bound[i];
    inequality_matrix(2 * i + 1, index) = 1.0;
    inequality_boundary(2 * i + 1, 0) = lower_bound[i];
  }
  inequality_matrices_.push_back(inequality_matrix);
  inequality_boundaries_.push_back(inequality_boundary);
  return true;
}

bool PiecewiseLinearConstraint::AddDerivativeBoundary(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  if (index_list.size() != lower_bound.size() ||
      index_list.size() != upper_bound.size()) {
    AERROR << "The sizes of index list, lower_bound, upper_bound are not "
              "identical.";
    return false;
  }
  Eigen::MatrixXd inequality_matrix =
      Eigen::MatrixXd::Zero(2 * index_list.size(), dimension_);
  Eigen::MatrixXd inequality_boundary =
      Eigen::MatrixXd::Zero(2 * index_list.size(), 1);

  for (uint32_t i = 0; i < index_list.size(); ++i) {
    uint32_t index = index_list[i];
    if (index == 0) {
      AERROR << "Index should NOT be 0.";
      return false;
    }

    inequality_matrix(2 * i, index - 1) = 1.0;
    inequality_matrix(2 * i, index) = -1.0;
    inequality_boundary(2 * i, 0) = -unit_segment_ * upper_bound[i];

    inequality_matrix(2 * i + 1, index - 1) = -1.0;
    inequality_matrix(2 * i + 1, index) = 1.0;
    inequality_boundary(2 * i, 0) = unit_segment_ * lower_bound[i];
  }
  inequality_matrices_.push_back(inequality_matrix);
  inequality_boundaries_.push_back(inequality_boundary);
  return true;
}

bool PiecewiseLinearConstraint::AddSecondDerivativeBoundary(
    const double init_derivative, const std::vector<uint32_t>& index_list,
    const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  // TODO(Liangliang): implement this function
  if (index_list.size() != lower_bound.size() ||
      index_list.size() != upper_bound.size()) {
    AERROR << "The sizes of index list, lower_bound, upper_bound are not "
              "identical.";
    return false;
  }
  Eigen::MatrixXd inequality_matrix =
      Eigen::MatrixXd::Zero(2 * index_list.size(), dimension_);
  Eigen::MatrixXd inequality_boundary =
      Eigen::MatrixXd::Zero(2 * index_list.size(), 1);

  for (uint32_t i = 0; i < index_list.size(); ++i) {
    uint32_t index = index_list[i];
    if (index == 0) {
      AERROR << "Index should NOT be 0.";
      return false;
    }

    const double upper = upper_bound[i];
    const double lower = lower_bound[i];
    if (index == 1) {
      inequality_matrix(2 * i, 1) = -1.0;
      inequality_boundary(2 * i, 1) = -(upper * unit_segment_ * unit_segment_ +
                                        init_derivative * unit_segment_);
      inequality_matrix(2 * i + 1, 1) = 1.0;
      inequality_boundary(2 * i + 1, 1) =
          lower * unit_segment_ * unit_segment_ +
          init_derivative * unit_segment_;
    } else {
      inequality_matrix(2 * i, index - 2) = -1.0;
      inequality_matrix(2 * i, index - 1) = 2.0;
      inequality_matrix(2 * i, index) = -1.0;
      inequality_boundary(2 * i, 0) = -upper * unit_segment_ * unit_segment_;

      inequality_matrix(2 * i + 1, index - 2) = 1.0;
      inequality_matrix(2 * i + 1, index - 1) = -2.0;
      inequality_matrix(2 * i + 1, index) = 1.0;
      inequality_boundary(2 * i + 1, 0) = lower * unit_segment_ * unit_segment_;
    }
  }
  inequality_matrices_.push_back(inequality_matrix);
  inequality_boundaries_.push_back(inequality_boundary);
  return true;
}

bool PiecewiseLinearConstraint::AddPointConstraint(const double x,
                                                   const double fx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddPointDerivativeConstraint(const double x,
                                                             const double dfx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddPointSecondDerivativeConstraint(
    const double x, const double ddfx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddPointThirdDerivativeConstraint(
    const double x, const double dddfx) {
  // TODO(Liangliang): implement this function
  return true;
}

bool PiecewiseLinearConstraint::AddMonotoneInequalityConstraint() {
  Eigen::MatrixXd inequality_matrix =
      Eigen::MatrixXd::Zero(dimension_ - 1, dimension_ - 1);
  Eigen::MatrixXd inequality_boundary =
      Eigen::MatrixXd::Zero(dimension_ - 1, 1);

  for (uint32_t i = 1; i < dimension_; ++i) {
    inequality_matrix(i, i - 1) = -1.0;
    inequality_matrix(i, i) = 1.0;
  }
  inequality_matrices_.push_back(inequality_matrix);
  inequality_boundaries_.push_back(inequality_boundary);

  return true;
}

}  // namespace planning
}  // namespace apollo
