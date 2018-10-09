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
 * @brief: Definition of PiecewiseLinearKernel class
 **/

#pragma once

#include <vector>

#include "Eigen/Core"

namespace apollo {
namespace planning {

class PiecewiseLinearKernel {
 public:
  PiecewiseLinearKernel(const uint32_t dimension, const double unit_segment);

  void AddRegularization(const double param);

  const Eigen::MatrixXd& kernel_matrix() const;
  const Eigen::MatrixXd& offset_matrix() const;

  void AddSecondOrderDerivativeMatrix(const double init_derivative,
                                      const double weight);
  void AddThirdOrderDerivativeMatrix(const double init_derivative,
                                     const double init_second_derivative,
                                     const double weight);

  // reference line kernel
  bool AddReferenceLineKernelMatrix(const std::vector<uint32_t>& index_list,
                                    const std::vector<double>& pos_list,
                                    const double weight);

 private:
  const uint32_t dimension_;
  const double unit_segment_;
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd offset_matrix_;
};

}  // namespace planning
}  // namespace apollo
