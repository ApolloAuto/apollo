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
 * @brief: The implementation of PiecewiseLinearKernel class.
 **/

#include "modules/planning/math/smoothing_spline/piecewise_linear_kernel.h"

namespace apollo {
namespace planning {

PiecewiseLinearKernel::PiecewiseLinearKernel(const uint32_t dimension)
    : dimension_(dimension),
      kernel_matrix_(Eigen::MatrixXd::Zero(dimension_, dimension_)),
      offset_matrix_(Eigen::MatrixXd::Zero(dimension_, 1)) {}

void PiecewiseLinearKernel::AddRegularization(const double regularized_param) {
  kernel_matrix_ +=
      Eigen::MatrixXd::Constant(dimension_, dimension_, regularized_param);
}

const Eigen::MatrixXd& PiecewiseLinearKernel::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& PiecewiseLinearKernel::offset_matrix() const {
  return offset_matrix_;
}

// build-in kernel methods
void PiecewiseLinearKernel::AddDerivativeKernelMatrix(const double weight) {
  // TODO(Liangliang): Implement this function.
}

void PiecewiseLinearKernel::AddSecondOrderDerivativeMatrix(
    const double weight) {
  // TODO(Liangliang): Implement this function.
}

void PiecewiseLinearKernel::AddThirdOrderDerivativeMatrix(const double weight) {
  // TODO(Liangliang): Implement this function.
}

// reference line kernel
bool PiecewiseLinearKernel::AddReferenceLineKernelMatrix(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& pos_list, const double weight) {
  // TODO(Liangliang): Implement this function.
  return true;
}

}  // namespace planning
}  // namespace apollo
