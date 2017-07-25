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

#ifndef MODULES_PLANNING_MATH_SMOOTHING_SPLINE_1D_CONSTRAINT_H_
#define MODULES_PLANNING_MATH_SMOOTHING_SPLINE_1D_CONSTRAINT_H_

#include <algorithm>
#include <vector>
#include "Eigen/Core"

#include "modules/planning/math/smoothing_spline/affine_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_1d.h"

namespace apollo {
namespace planning {

class Spline1dConstraint {
 public:
  explicit Spline1dConstraint(const Spline1d& pss);
  Spline1dConstraint(const std::vector<double>& x_knots,
                     const std::uint32_t order);

  // direct methods
  bool add_inequality_constraint(const Eigen::MatrixXd& constraint_matrix,
                                 const Eigen::MatrixXd& constraint_boundary);
  bool add_equality_constraint(const Eigen::MatrixXd& constraint_matrix,
                               const Eigen::MatrixXd& constraint_boundary);

  // preset method
  /**
  *   @brief: inequality boundary constraints
  *   if no boundary, do specify either by std::infinity or let vector.size() =
  *0
  **/
  bool add_fx_boundary(const std::vector<double>& x_coord,
                       const std::vector<double>& lower_bound,
                       const std::vector<double>& upper_bound);

  bool add_derivative_boundary(const std::vector<double>& x_coord,
                               const std::vector<double>& lower_bound,
                               const std::vector<double>& upper_bound);

  bool add_second_derivative_boundary(const std::vector<double>& x_coord,
                                      const std::vector<double>& lower_bound,
                                      const std::vector<double>& upper_bound);

  bool add_third_derivative_boundary(const std::vector<double>& x_coord,
                                     const std::vector<double>& lower_bound,
                                     const std::vector<double>& upper_bound);

  /**
  *   @brief: equality constraint to guarantee joint smoothness
  **/
  // boundary equality constriant
  // constraint on fx, dfx, ddfx ... in vector form; upto third order
  bool add_point_fx_constraint(const double x, const double fx);
  bool add_point_derivative_constraint(const double x, const double dfx);
  bool add_point_second_derivative_constraint(const double x,
                                              const double ddfx);
  bool add_point_third_derivative_constraint(const double x,
                                             const double dddfx);

  // guarantee upto values are joint
  bool add_fx_smooth_constraint();

  // guarantee upto derivative are joint
  bool add_derivative_smooth_constraint();

  // guarantee upto second order derivative are joint
  bool add_second_derivative_smooth_constraint();

  // guarantee upto third order derivative are joint
  bool add_third_derivative_smooth_constraint();

  /**
  *   @brief: Add monotone constraint inequality, guarantee the monotone city at
  *evaluated point
  **/

  // customized monotone inequality constraint at x_coord
  bool add_monotone_fx_inequality_constraint(
      const std::vector<double>& x_coord);

  // default inequality constraint at knots
  bool add_monotone_fx_inequality_constraint_at_knots();

  /**
  *   @brief: output interface inequality constraint
  **/
  const AffineConstraint& inequality_constraint() const;
  const AffineConstraint& equality_constraint() const;

 private:
  std::uint32_t find_index(const double x) const;

  bool filter_constraints(const std::vector<double>& x_coord,
                          const std::vector<double>& lower_bound,
                          const std::vector<double>& upper_bound,
                          std::vector<double>* const filtered_lower_bound_x,
                          std::vector<double>* const filtered_lower_bound,
                          std::vector<double>* const filtered_upper_bound_x,
                          std::vector<double>* const filtered_upper_bound);
  void generate_power_x(const double x, const std::uint32_t order,
                        std::vector<double>* const power_x) const;

 private:
  AffineConstraint inequality_constraint_;
  AffineConstraint equality_constraint_;
  std::vector<double> x_knots_;
  std::uint32_t spline_order_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SMOOTHING_SPLINE_1D_CONSTRAINT_H_
