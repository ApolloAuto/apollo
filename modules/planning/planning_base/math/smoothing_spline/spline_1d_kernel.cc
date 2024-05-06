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
 * @file : piecewise_smoothing_spline_constraint.h
 * @brief: wrap up solver constraint interface with direct methods and preset
 *methods
 **/

#include "modules/planning/planning_base/math/smoothing_spline/spline_1d_kernel.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/math/smoothing_spline/spline_seg_kernel.h"

namespace apollo {
namespace planning {

Spline1dKernel::Spline1dKernel(const Spline1d& spline1d)
    : Spline1dKernel(spline1d.x_knots(), spline1d.spline_order()) {}

Spline1dKernel::Spline1dKernel(const std::vector<double>& x_knots,
                               const uint32_t spline_order)
    : x_knots_(x_knots), spline_order_(spline_order) {
  total_params_ =
      (static_cast<uint32_t>(x_knots.size()) > 1
           ? (static_cast<uint32_t>(x_knots.size()) - 1) * (1 + spline_order_)
           : 0);
  kernel_matrix_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
  offset_ = Eigen::MatrixXd::Zero(total_params_, 1);
}

void Spline1dKernel::AddRegularization(const double regularized_param) {
  Eigen::MatrixXd id_matrix =
      Eigen::MatrixXd::Identity(kernel_matrix_.rows(), kernel_matrix_.cols());
  kernel_matrix_ += 2.0 * id_matrix * regularized_param;
}

bool Spline1dKernel::AddKernel(const Eigen::MatrixXd& kernel,
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

bool Spline1dKernel::AddKernel(const Eigen::MatrixXd& kernel,
                               const double weight) {
  Eigen::MatrixXd offset = Eigen::MatrixXd::Zero(kernel.rows(), 1);
  return AddKernel(kernel, offset, weight);
}

Eigen::MatrixXd* Spline1dKernel::mutable_kernel_matrix() {
  return &kernel_matrix_;
}

Eigen::MatrixXd* Spline1dKernel::mutable_offset() { return &offset_; }

const Eigen::MatrixXd& Spline1dKernel::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& Spline1dKernel::offset() const { return offset_; }

// build-in kernel methods
void Spline1dKernel::AddNthDerivativekernelMatrix(const uint32_t n,
                                                  const double weight) {
  const uint32_t num_params = spline_order_ + 1;
  for (uint32_t i = 0; i + 1 < x_knots_.size(); ++i) {
    Eigen::MatrixXd cur_kernel =
        2 *
        SplineSegKernel::Instance()->NthDerivativeKernel(
            n, num_params, x_knots_[i + 1] - x_knots_[i]) *
        weight;
    kernel_matrix_.block(i * num_params, i * num_params, num_params,
                         num_params) += cur_kernel;
  }
}

void Spline1dKernel::AddDerivativeKernelMatrix(const double weight) {
  AddNthDerivativekernelMatrix(1, weight);
}

void Spline1dKernel::AddSecondOrderDerivativeMatrix(const double weight) {
  AddNthDerivativekernelMatrix(2, weight);
}

void Spline1dKernel::AddThirdOrderDerivativeMatrix(const double weight) {
  AddNthDerivativekernelMatrix(3, weight);
}

void Spline1dKernel::AddNthDerivativekernelMatrixForSplineK(
    const uint32_t n, const uint32_t k, const double weight) {
  if (k + 1 >= x_knots_.size()) {
    AERROR << "Cannot add NthDerivativeKernel for spline K because k is out of "
              "range. k = "
           << k;
    return;
  }
  const uint32_t num_params = spline_order_ + 1;
  Eigen::MatrixXd cur_kernel =
      2 *
      SplineSegKernel::Instance()->NthDerivativeKernel(
          n, num_params, x_knots_[k + 1] - x_knots_[k]) *
      weight;
  kernel_matrix_.block(k * num_params, k * num_params, num_params,
                       num_params) += cur_kernel;
}

void Spline1dKernel::AddDerivativeKernelMatrixForSplineK(const uint32_t k,
                                                         const double weight) {
  AddNthDerivativekernelMatrixForSplineK(1, k, weight);
}

void Spline1dKernel::AddSecondOrderDerivativeMatrixForSplineK(
    const uint32_t k, const double weight) {
  AddNthDerivativekernelMatrixForSplineK(2, k, weight);
}

void Spline1dKernel::AddThirdOrderDerivativeMatrixForSplineK(
    const uint32_t k, const double weight) {
  AddNthDerivativekernelMatrixForSplineK(3, k, weight);
}

bool Spline1dKernel::AddReferenceLineKernelMatrix(
    const std::vector<double>& x_coord, const std::vector<double>& ref_x,
    const double weight) {
  if (ref_x.size() != x_coord.size()) {
    return false;
  }

  const uint32_t num_params = spline_order_ + 1;
  for (size_t i = 0; i < x_coord.size(); ++i) {
    auto cur_index = FindIndex(x_coord[i]);
    double cur_rel_x = x_coord[i] - x_knots_[cur_index];
    // update offset
    double offset_coef = -2.0 * ref_x[i] * weight;
    for (size_t j = 0; j < num_params; ++j) {
      offset_(j + cur_index * num_params, 0) += offset_coef;
      offset_coef *= cur_rel_x;
    }
    // update kernel matrix
    Eigen::MatrixXd ref_kernel(num_params, num_params);

    double cur_x = 1.0;
    std::vector<double> power_x;
    for (uint32_t n = 0; n + 1 < 2 * num_params; ++n) {
      power_x.emplace_back(cur_x);
      cur_x *= cur_rel_x;
    }

    for (uint32_t r = 0; r < num_params; ++r) {
      for (uint32_t c = 0; c < num_params; ++c) {
        ref_kernel(r, c) = 2.0 * power_x[r + c];
      }
    }

    kernel_matrix_.block(cur_index * num_params, cur_index * num_params,
                         num_params, num_params) += weight * ref_kernel;
  }
  return true;
}

uint32_t Spline1dKernel::FindIndex(const double x) const {
  auto upper_bound = std::upper_bound(x_knots_.begin() + 1, x_knots_.end(), x);
  return std::min(static_cast<uint32_t>(x_knots_.size() - 1),
                  static_cast<uint32_t>(upper_bound - x_knots_.begin())) -
         1;
}

void Spline1dKernel::AddDistanceOffset(const double weight) {
  for (uint32_t i = 1; i < x_knots_.size(); ++i) {
    const double cur_x = x_knots_[i] - x_knots_[i - 1];
    double pw_x = 2.0 * weight;
    for (uint32_t j = 0; j < spline_order_ + 1; ++j) {
      offset_((i - 1) * (spline_order_ + 1) + j, 0) -= pw_x;
      pw_x *= cur_x;
    }
  }
}
}  // namespace planning
}  // namespace apollo
