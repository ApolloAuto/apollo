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
 * @file : spline_1d_kernel.h
 * @brief: wrap up solver constraint interface with direct methods and preset
 *methods
 **/

#pragma once

#include <vector>

#include "Eigen/Core"

#include "modules/planning/planning_base/math/smoothing_spline/spline_1d.h"

namespace apollo {
namespace planning {

class Spline1dKernel {
 public:
  explicit Spline1dKernel(const Spline1d& spline1d);
  Spline1dKernel(const std::vector<double>& x_knots,
                 const uint32_t spline_order);

  // customized input / output method
  void AddRegularization(const double regularized_param);
  bool AddKernel(const Eigen::MatrixXd& kernel, const Eigen::MatrixXd& offset,
                 const double weight);
  bool AddKernel(const Eigen::MatrixXd& kernel, const double weight);

  Eigen::MatrixXd* mutable_kernel_matrix();
  Eigen::MatrixXd* mutable_offset();

  const Eigen::MatrixXd& kernel_matrix() const;
  const Eigen::MatrixXd& offset() const;

  // build-in kernel methods
  void AddDerivativeKernelMatrix(const double weight);
  void AddSecondOrderDerivativeMatrix(const double weight);
  void AddThirdOrderDerivativeMatrix(const double weight);
  void AddDerivativeKernelMatrixForSplineK(const uint32_t k,
                                           const double weight);
  void AddSecondOrderDerivativeMatrixForSplineK(const uint32_t k,
                                                const double weight);
  void AddThirdOrderDerivativeMatrixForSplineK(const uint32_t k,
                                               const double weight);

  // reference line kernel, x_coord in strictly increasing order (for path
  // optimizer)
  bool AddReferenceLineKernelMatrix(const std::vector<double>& x_coord,
                                    const std::vector<double>& ref_fx,
                                    const double weight);

  // distance offset (for speed optimizer, given time optimize the distance can
  // go)
  void AddDistanceOffset(const double weight);

 private:
  void AddNthDerivativekernelMatrix(const uint32_t n, const double weight);
  void AddNthDerivativekernelMatrixForSplineK(const uint32_t n,
                                              const uint32_t k,
                                              const double weight);
  uint32_t FindIndex(const double x) const;

 private:
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd offset_;
  std::vector<double> x_knots_;
  uint32_t spline_order_;
  uint32_t total_params_;
};

}  // namespace planning
}  // namespace apollo
