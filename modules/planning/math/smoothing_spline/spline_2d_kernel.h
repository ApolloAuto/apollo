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
 * @file spline_2d_kernel.h
 **/

#pragma once

#include <vector>

#include "Eigen/Core"
#include "modules/common/math/vec2d.h"
#include "modules/planning/math/smoothing_spline/spline_2d.h"

namespace apollo {
namespace planning {

class Spline2dKernel {
 public:
  Spline2dKernel(const std::vector<double>& t_knots,
                 const uint32_t spline_order);

  // customized input output
  void AddRegularization(const double regularization_param);
  bool AddKernel(const Eigen::MatrixXd& kernel, const Eigen::MatrixXd& offset,
                 const double weight);
  bool AddKernel(const Eigen::MatrixXd& kernel, const double weight);

  Eigen::MatrixXd* mutable_kernel_matrix();
  Eigen::MatrixXd* mutable_offset();

  Eigen::MatrixXd kernel_matrix() const;
  const Eigen::MatrixXd offset() const;

  // build-in kernel methods
  void AddDerivativeKernelMatrix(const double weight);
  void AddSecondOrderDerivativeMatrix(const double weight);
  void AddThirdOrderDerivativeMatrix(const double weight);

  // reference line kernel, x_coord in strictly increasing order
  bool AddReferenceLineKernelMatrix(
      const std::vector<double>& t_coord,
      const std::vector<common::math::Vec2d>& ref_points, const double weight);

 private:
  void AddNthDerivativeKernelMatrix(const uint32_t n, const double weight);
  uint32_t find_index(const double x) const;

 private:
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd offset_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
  size_t total_params_;
};

}  // namespace planning
}  // namespace apollo
