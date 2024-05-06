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
 * @file : spline_1d_constraint.h
 * @brief: wrapp up solver constraint interface with direct methods and preset
 *methods
 **/

#pragma once

#include <algorithm>
#include <vector>

#include "Eigen/Core"

#include "modules/planning/planning_base/math/smoothing_spline/affine_constraint.h"
#include "modules/planning/planning_base/math/smoothing_spline/spline_1d.h"

namespace apollo {
namespace planning {

class Spline1dConstraint {
 public:
  explicit Spline1dConstraint(const Spline1d& pss);
  Spline1dConstraint(const std::vector<double>& x_knots, const uint32_t order);

  // direct methods
  bool AddInequalityConstraint(const Eigen::MatrixXd& constraint_matrix,
                               const Eigen::MatrixXd& constraint_boundary);
  bool AddEqualityConstraint(const Eigen::MatrixXd& constraint_matrix,
                             const Eigen::MatrixXd& constraint_boundary);

  // preset method
  /**
   * @brief: inequality boundary constraints
   * if no boundary, do specify either by std::infinity or
   * let vector.size() = 0
   **/
  bool AddBoundary(const std::vector<double>& x_coord,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);

  bool AddDerivativeBoundary(const std::vector<double>& x_coord,
                             const std::vector<double>& lower_bound,
                             const std::vector<double>& upper_bound);

  bool AddSecondDerivativeBoundary(const std::vector<double>& x_coord,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound);

  bool AddThirdDerivativeBoundary(const std::vector<double>& x_coord,
                                  const std::vector<double>& lower_bound,
                                  const std::vector<double>& upper_bound);

  /**
   * @brief: equality constraint to guarantee joint smoothness
   * boundary equality constriant constraint on fx, dfx, ddfx ... in vector
   * form; up to third order
   **/
  bool AddPointConstraint(const double x, const double fx);
  bool AddPointDerivativeConstraint(const double x, const double dfx);
  bool AddPointSecondDerivativeConstraint(const double x, const double ddfx);
  bool AddPointThirdDerivativeConstraint(const double x, const double dddfx);

  bool AddPointConstraintInRange(const double x, const double fx,
                                 const double range);
  bool AddPointDerivativeConstraintInRange(const double x, const double dfx,
                                           const double range);
  bool AddPointSecondDerivativeConstraintInRange(const double x,
                                                 const double ddfx,
                                                 const double range);
  bool AddPointThirdDerivativeConstraintInRange(const double x,
                                                const double dddfx,
                                                const double range);
  // guarantee up to values are joint
  bool AddSmoothConstraint();

  // guarantee up to derivative are joint
  bool AddDerivativeSmoothConstraint();

  // guarantee up to second order derivative are joint
  bool AddSecondDerivativeSmoothConstraint();

  // guarantee up to third order derivative are joint
  bool AddThirdDerivativeSmoothConstraint();

  /**
   * @brief: Add monotone constraint inequality, guarantee the monotone city at
   * evaluated point. customized monotone inequality constraint at x_coord
   **/
  bool AddMonotoneInequalityConstraint(const std::vector<double>& x_coord);

  // default inequality constraint at knots
  bool AddMonotoneInequalityConstraintAtKnots();

  /**
   * @brief: output interface inequality constraint
   **/
  const AffineConstraint& inequality_constraint() const;
  const AffineConstraint& equality_constraint() const;

 private:
  uint32_t FindIndex(const double x) const;

  bool FilterConstraints(const std::vector<double>& x_coord,
                         const std::vector<double>& lower_bound,
                         const std::vector<double>& upper_bound,
                         std::vector<double>* const filtered_lower_bound_x,
                         std::vector<double>* const filtered_lower_bound,
                         std::vector<double>* const filtered_upper_bound_x,
                         std::vector<double>* const filtered_upper_bound);
  void GeneratePowerX(const double x, const uint32_t order,
                      std::vector<double>* const power_x) const;

  using AddConstraintInRangeFunc =
      std::function<bool(const std::vector<double>&, const std::vector<double>&,
                         const std::vector<double>&)>;
  bool AddConstraintInRange(AddConstraintInRangeFunc func, const double x,
                            const double val, const double range);

 private:
  AffineConstraint inequality_constraint_;
  AffineConstraint equality_constraint_;
  std::vector<double> x_knots_;
  uint32_t spline_order_;
};

}  // namespace planning
}  // namespace apollo
