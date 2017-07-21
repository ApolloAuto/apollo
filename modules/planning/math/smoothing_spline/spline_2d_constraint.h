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
 * @file spline_2d_constraint.h
 **/

#ifndef MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_CONSTRAINT_H_
#define MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_CONSTRAINT_H_

#include <vector>
#include "Eigen/Core"

#include "modules/common/math/vec2d_utils.h"
#include "modules/planning/math/smoothing_spline/affine_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_2d.h"

namespace apollo {
namespace planning {

class Spline2dConstraint {
 public:
  Spline2dConstraint(const std::vector<double>& t_knots,
                     const std::size_t order);

  // direct method
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
  bool add_2d_boundary(
      const std::vector<double>& t_coord, const std::vector<double>& angle,
      const std::vector<apollo::common::Vec2D>& ref_point,
      const std::vector<double>& longitidinal_bound,
      const std::vector<double>& lateral_bound);

  // ref point refer to derivative reference point
  bool add_2d_derivative_boundary(
      const std::vector<double>& t_coord, const std::vector<double>& angle,
      const std::vector<apollo::common::Vec2D>& ref_point,
      const std::vector<double>& longitidinal_bound,
      const std::vector<double>& lateral_bound);

  // ref point refer to second derivative ref point
  bool add_2d_second_derivative_boundary(
      const std::vector<double>& t_coord, const std::vector<double>& angle,
      const std::vector<apollo::common::Vec2D>& ref_point,
      const std::vector<double>& longitidinal_bound,
      const std::vector<double>& lateral_bound);

  // ref point refer to third derivative ref point
  bool add_2d_third_derivative_boundary(
      const std::vector<double>& t_coord, const std::vector<double>& angle,
      const std::vector<apollo::common::Vec2D>& ref_point,
      const std::vector<double>& longitidinal_bound,
      const std::vector<double>& lateral_bound);

  bool add_point_constraint(const double t, const double x, const double y);
  bool add_point_angle_constraint(const double t, const double angle);

  // guarantee upto values are joint
  bool add_fx_smooth_constraint();

  // guarantee upto derivative are joint
  bool add_derivative_smooth_constraint();

  // guarantee upto second order derivative are joint
  bool add_second_derivative_smooth_constraint();

  // guarantee upto third order derivative are joint
  bool add_third_derivative_smooth_constraint();

  /**
  *   @brief: output interface inequality constraint
  **/
  const AffineConstraint& inequality_constraint() const;
  const AffineConstraint& equality_constraint() const;

 private:
  std::size_t find_index(const double t) const;
  std::vector<double> affine_coef(const double angle, const double t) const;
  std::vector<double> affine_derivative_coef(const double angle,
                                             const double t) const;
  std::vector<double> affine_second_derivative_coef(const double angle,
                                                    const double t) const;
  std::vector<double> affine_third_derivative_coef(const double angle,
                                                   const double t) const;
  std::vector<double> poly_coef(const double t) const;
  std::vector<double> derivative_coef(const double t) const;
  std::vector<double> second_derivative_coef(const double t) const;
  std::vector<double> third_derivative_coef(const double t) const;
  double sign_distance(const apollo::common::Vec2D& xy_point,
                       const double angle) const;

 private:
  AffineConstraint inequality_constraint_;
  AffineConstraint equality_constraint_;
  std::vector<double> t_knots_;
  std::size_t spline_order_;
  std::size_t total_param_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_SMOOTHING_SPLINE_SPLINE_2D_CONSTRAINT_H_
