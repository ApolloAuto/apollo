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
 * @file : spline_seg_kernel.h
 * @brief: generating integrated kernels for smoothing spline
 *
 *           x' P x  = int_0 ^x  (f(x)^(k))^2 dx, k = 0, 1, 2, 3
 *           P is the kernel of k-th smooth kernel
 **/

#ifndef MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_SEG_KERNEL_H_
#define MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_SEG_KERNEL_H_

#include <cstddef>
#include <string>

#include "Eigen/Core"

#include "modules/common/macro.h"

namespace apollo {
namespace planning {

class SplineSegKernel {
 public:
  // generating kernel matrix
  Eigen::MatrixXd Kernel(const std::uint32_t order, const double accumalated_x);
  Eigen::MatrixXd DerivativeKernel(const std::uint32_t order,
                                   const double accumalated_x);
  Eigen::MatrixXd SecondOrderDerivativeKernel(const std::uint32_t order,
                                              const double accumalated_x);
  Eigen::MatrixXd ThirdOrderDerivativeKernel(const std::uint32_t order,
                                             const double accumalated_x);

 private:
  void integrated_term_matrix(const std::uint32_t order, const double x,
                              const std::string& type,
                              Eigen::MatrixXd* term_matrix) const;
  void calculate_fx(const std::uint32_t order);
  void CalculateDerivative(const std::uint32_t order);
  void CalculateSecondOrderDerivative(const std::uint32_t order);
  void CalculateThirdOrderDerivative(const std::uint32_t order);

  std::uint32_t reserved_order_;
  Eigen::MatrixXd kernel_fx_;
  Eigen::MatrixXd kernel_derivative_;
  Eigen::MatrixXd kernel_second_order_derivative_;
  Eigen::MatrixXd kernel_third_order_derivative_;
  DECLARE_SINGLETON(SplineSegKernel);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_SEG_KERNEL_H_
