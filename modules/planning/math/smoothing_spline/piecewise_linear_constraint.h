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
 * @file : piecewise_linear_constraint.h
 * @brief: Definition of PiecewiseLinearConstraint class.
 **/

#pragma once

#include <vector>

#include "Eigen/Core"

namespace apollo {
namespace planning {

class PiecewiseLinearConstraint {
 public:
  PiecewiseLinearConstraint(const uint32_t dimension,
                            const double unit_segment);
  virtual ~PiecewiseLinearConstraint() = default;

  Eigen::MatrixXd inequality_constraint_matrix() const;
  Eigen::MatrixXd inequality_constraint_boundary() const;
  Eigen::MatrixXd equality_constraint_matrix() const;
  Eigen::MatrixXd equality_constraint_boundary() const;

  /**
   * @brief: inequality boundary constraints
   **/
  bool AddBoundary(const std::vector<uint32_t>& index_list,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);
  bool AddDerivativeBoundary(const std::vector<uint32_t>& index_list,
                             const std::vector<double>& lower_bound,
                             const std::vector<double>& upper_bound);
  bool AddSecondDerivativeBoundary(const double init_derivative,
                                   const std::vector<uint32_t>& index_list,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound);

  /**
   * @brief: equality constraints
   **/
  bool AddPointConstraint(const uint32_t index, const double val);
  bool AddPointDerivativeConstraint(const uint32_t index, const double val);

  /**
   * @brief: Add monotone constraint inequality at all indices
   **/
  bool AddMonotoneInequalityConstraint();

 private:
  const uint32_t dimension_;
  const double unit_segment_;
  std::vector<Eigen::MatrixXd> inequality_matrices_;
  std::vector<Eigen::MatrixXd> inequality_boundaries_;

  std::vector<Eigen::MatrixXd> equality_matrices_;
  std::vector<Eigen::MatrixXd> equality_boundaries_;
};

}  // namespace planning
}  // namespace apollo
