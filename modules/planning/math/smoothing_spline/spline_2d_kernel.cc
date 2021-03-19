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
 * @file spline_2d_kernel.cc
 **/

#include "modules/planning/math/smoothing_spline/spline_2d_kernel.h"

#include <algorithm>

#include "modules/planning/math/smoothing_spline/spline_seg_kernel.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

Spline2dKernel::Spline2dKernel(const std::vector<double>& t_knots,
                               const uint32_t spline_order)
    : t_knots_(t_knots), spline_order_(spline_order) {
  total_params_ =
      (t_knots_.size() > 1 ? 2 * (t_knots_.size() - 1) * (1 + spline_order_)
                           : 0);
  kernel_matrix_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
  offset_ = Eigen::MatrixXd::Zero(total_params_, 1);
}

// customized input output
void Spline2dKernel::AddRegularization(const double regularization_param) {
  Eigen::MatrixXd id_matrix =
      Eigen::MatrixXd::Identity(kernel_matrix_.rows(), kernel_matrix_.cols());
  kernel_matrix_ += id_matrix * regularization_param;
}

bool Spline2dKernel::AddKernel(const Eigen::MatrixXd& kernel,
                               const Eigen::MatrixXd& offset,
                               const double weight) {
  if (kernel.rows() != kernel.cols() ||
      kernel.rows() != kernel_matrix_.rows() || offset.cols() != 1 ||
      offset.rows() != offset_.rows()) {
    return false;
  }
  kernel_matrix_ += kernel * weight;
  offset_ += offset * weight;
  return true;
}

bool Spline2dKernel::AddKernel(const Eigen::MatrixXd& kernel,
                               const double weight) {
  Eigen::MatrixXd offset = Eigen::MatrixXd::Zero(kernel.rows(), 1);
  return AddKernel(kernel, offset, weight);
}

Eigen::MatrixXd* Spline2dKernel::mutable_kernel_matrix() {
  return &kernel_matrix_;
}

Eigen::MatrixXd* Spline2dKernel::mutable_offset() { return &offset_; }

Eigen::MatrixXd Spline2dKernel::kernel_matrix() const {
  return kernel_matrix_ * 2.0;
}

const Eigen::MatrixXd Spline2dKernel::offset() const { return offset_; }

// build-in kernel methods

void Spline2dKernel::AddNthDerivativeKernelMatrix(const uint32_t n,
                                                  const double weight) {
  for (uint32_t i = 0; i + 1 < t_knots_.size(); ++i) {
    const uint32_t num_params = spline_order_ + 1;
    Eigen::MatrixXd cur_kernel =
        SplineSegKernel::Instance()->NthDerivativeKernel(
            n, num_params, t_knots_[i + 1] - t_knots_[i]) *
        weight;
    kernel_matrix_.block(2 * i * num_params, 2 * i * num_params, num_params,
                         num_params) += cur_kernel;
    kernel_matrix_.block((2 * i + 1) * num_params, (2 * i + 1) * num_params,
                         num_params, num_params) += cur_kernel;
  }
}

void Spline2dKernel::AddDerivativeKernelMatrix(const double weight) {
  AddNthDerivativeKernelMatrix(1, weight);
}

void Spline2dKernel::AddSecondOrderDerivativeMatrix(const double weight) {
  AddNthDerivativeKernelMatrix(2, weight);
}

void Spline2dKernel::AddThirdOrderDerivativeMatrix(const double weight) {
  AddNthDerivativeKernelMatrix(3, weight);
}

// reference line kernel, t_coord in strictly increasing order (for path
// optimizer)
bool Spline2dKernel::AddReferenceLineKernelMatrix(
    const std::vector<double>& t_coord, const std::vector<Vec2d>& ref_points,
    const double weight) {
  if (ref_points.size() != t_coord.size()) {
    return false;
  }

  for (uint32_t i = 0; i < t_coord.size(); ++i) {
    auto cur_index = find_index(t_coord[i]);
    double cur_rel_t = t_coord[i] - t_knots_[cur_index];
    // update offset
    double offset_coef_x = -ref_points[i].x() * weight;
    double offset_coef_y = -ref_points[i].y() * weight;
    const uint32_t num_params = spline_order_ + 1;
    for (uint32_t j = 0; j < num_params; ++j) {
      offset_(j + (2 * cur_index) * num_params, 0) = offset_coef_x;
      offset_(j + (2 * cur_index + 1) * num_params, 0) = offset_coef_y;
      offset_coef_x *= cur_rel_t;
      offset_coef_y *= cur_rel_t;
    }

    // update kernel matrix
    Eigen::MatrixXd ref_kernel(num_params, num_params);

    double cur_t = 1.0;
    std::vector<double> power_t;
    for (uint32_t n = 0; n + 1 < 2 * num_params; ++n) {
      power_t.emplace_back(cur_t);
      cur_t *= cur_rel_t;
    }

    for (uint32_t r = 0; r < num_params; ++r) {
      for (uint32_t c = 0; c < num_params; ++c) {
        ref_kernel(r, c) = power_t[r + c];
      }
    }
    kernel_matrix_.block((2 * cur_index) * num_params,
                         (2 * cur_index) * num_params, num_params,
                         num_params) += weight * ref_kernel;
    kernel_matrix_.block((2 * cur_index + 1) * num_params,
                         (2 * cur_index + 1) * num_params, num_params,
                         num_params) += weight * ref_kernel;
  }
  return true;
}

uint32_t Spline2dKernel::find_index(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin() + 1, t_knots_.end(), t);
  return std::min(static_cast<uint32_t>(t_knots_.size() - 1),
                  static_cast<uint32_t>(upper_bound - t_knots_.begin())) -
         1;
}

}  // namespace planning
}  // namespace apollo
