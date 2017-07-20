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
 * @file : spline_smoother_constraint.cc
 **/

#include "modules/planning/math/smoothing_spline/spline_2d_constraint.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

Spline2dConstraint::Spline2dConstraint(const std::vector<double>& t_knots,
                                       const std::size_t order)
    : t_knots_(t_knots), spline_order_(order) {
  inequality_constraint_.set_is_equality(false);
  equality_constraint_.set_is_equality(true);
  total_param_ = 2 * spline_order_ * (t_knots.size() - 1);
}

// direct method
bool Spline2dConstraint::add_inequality_constraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  return inequality_constraint_.add_constraint(constraint_matrix,
                                               constraint_boundary);
}

bool Spline2dConstraint::add_equality_constraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  return equality_constraint_.add_constraint(constraint_matrix,
                                             constraint_boundary);
}

// preset method
/**
*   @brief: inequality boundary constraints
*   if no boundary, do specify either by std::infinity or let vector.size() = 0
**/
bool Spline2dConstraint::add_2d_boundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitidinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != longitidinal_bound.size() ||
      longitidinal_bound.size() != lateral_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (std::size_t i = 0; i < t_coord.size(); ++i) {
    const double d_longitudinal = sign_distance(ref_point[i], angle[i]);
    const double d_lateral = sign_distance(ref_point[i], angle[i] - M_PI / 2.0);
    const std::size_t index = find_index(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const std::size_t index_offset = 2 * index * spline_order_;
    std::vector<double> longi_coef = affine_coef(angle[i], rel_t);
    std::vector<double> lateral_coef = affine_coef(angle[i] - M_PI / 2, rel_t);
    for (std::size_t j = 0; j < 2 * spline_order_; ++j) {
      // upper longi
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper lateral
      affine_inequality(4 * i + 2, index_offset + j) = lateral_coef[j];
      // lower lateral
      affine_inequality(4 * i + 3, index_offset + j) = -lateral_coef[j];
    }

    affine_boundary(4 * i, 0) = d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 1, 0) = -d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 2, 0) = d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 3, 0) = -d_lateral - lateral_bound[i];
  }
  return add_inequality_constraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::add_2d_derivative_boundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitidinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != longitidinal_bound.size() ||
      longitidinal_bound.size() != lateral_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (std::size_t i = 0; i < t_coord.size(); ++i) {
    const double d_longitudinal = sign_distance(ref_point[i], angle[i]);
    const double d_lateral = sign_distance(ref_point[i], angle[i] - M_PI / 2.0);
    const std::size_t index = find_index(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const std::size_t index_offset = 2 * index * spline_order_;
    std::vector<double> longi_coef = affine_derivative_coef(angle[i], rel_t);
    std::vector<double> lateral_coef =
        affine_derivative_coef(angle[i] - M_PI / 2, rel_t);
    for (std::size_t j = 0; j < 2 * spline_order_; ++j) {
      // upper longi
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper lateral
      affine_inequality(4 * i + 2, index_offset + j) = lateral_coef[j];
      // lower lateral
      affine_inequality(4 * i + 3, index_offset + j) = -lateral_coef[j];
    }

    affine_boundary(4 * i, 0) = d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 1, 0) = -d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 2, 0) = d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 3, 0) = -d_lateral - lateral_bound[i];
  }
  return add_inequality_constraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::add_2d_second_derivative_boundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitidinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != longitidinal_bound.size() ||
      longitidinal_bound.size() != lateral_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (std::size_t i = 0; i < t_coord.size(); ++i) {
    const double d_longitudinal = sign_distance(ref_point[i], angle[i]);
    const double d_lateral = sign_distance(ref_point[i], angle[i] - M_PI / 2.0);
    const std::size_t index = find_index(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const std::size_t index_offset = 2 * index * spline_order_;
    std::vector<double> longi_coef =
        affine_second_derivative_coef(angle[i], rel_t);
    std::vector<double> lateral_coef =
        affine_second_derivative_coef(angle[i] - M_PI / 2, rel_t);
    for (std::size_t j = 0; j < 2 * spline_order_; ++j) {
      // upper longi
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper lateral
      affine_inequality(4 * i + 2, index_offset + j) = lateral_coef[j];
      // lower lateral
      affine_inequality(4 * i + 3, index_offset + j) = -lateral_coef[j];
    }

    affine_boundary(4 * i, 0) = d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 1, 0) = -d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 2, 0) = d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 3, 0) = -d_lateral - lateral_bound[i];
  }
  return add_inequality_constraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::add_2d_third_derivative_boundary(
    const std::vector<double>& t_coord, const std::vector<double>& angle,
    const std::vector<Vec2d>& ref_point,
    const std::vector<double>& longitidinal_bound,
    const std::vector<double>& lateral_bound) {
  if (t_coord.size() != angle.size() || angle.size() != ref_point.size() ||
      ref_point.size() != longitidinal_bound.size() ||
      longitidinal_bound.size() != lateral_bound.size()) {
    return false;
  }
  Eigen::MatrixXd affine_inequality =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * t_coord.size(), 1);
  for (std::size_t i = 0; i < t_coord.size(); ++i) {
    const double d_longitudinal = sign_distance(ref_point[i], angle[i]);
    const double d_lateral = sign_distance(ref_point[i], angle[i] - M_PI / 2.0);
    const std::size_t index = find_index(t_coord[i]);
    const double rel_t = t_coord[i] - t_knots_[index];
    const std::size_t index_offset = 2 * index * spline_order_;
    std::vector<double> longi_coef =
        affine_third_derivative_coef(angle[i], rel_t);
    std::vector<double> lateral_coef =
        affine_third_derivative_coef(angle[i] - M_PI / 2, rel_t);
    for (std::size_t j = 0; j < 2 * spline_order_; ++j) {
      // upper longi
      affine_inequality(4 * i, index_offset + j) = longi_coef[j];
      // lower longi
      affine_inequality(4 * i + 1, index_offset + j) = -longi_coef[j];
      // upper lateral
      affine_inequality(4 * i + 2, index_offset + j) = lateral_coef[j];
      // lower lateral
      affine_inequality(4 * i + 3, index_offset + j) = -lateral_coef[j];
    }

    affine_boundary(4 * i, 0) = d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 1, 0) = -d_longitudinal - longitidinal_bound[i];
    affine_boundary(4 * i + 2, 0) = d_lateral - lateral_bound[i];
    affine_boundary(4 * i + 3, 0) = -d_lateral - lateral_bound[i];
  }
  return add_inequality_constraint(affine_inequality, affine_boundary);
}

bool Spline2dConstraint::add_point_constraint(const double t, const double x,
                                              const double y) {
  const std::size_t index = find_index(t);
  const std::size_t index_offset = index * 2 * spline_order_;
  const double rel_t = t - t_knots_[index];

  Eigen::MatrixXd affine_equality = Eigen::MatrixXd::Zero(2, total_param_);
  Eigen::MatrixXd affine_boundary = Eigen::MatrixXd::Zero(2, 1);
  affine_boundary << x, y;
  std::vector<double> power_t = poly_coef(rel_t);
  for (std::size_t i = 0; i < spline_order_; ++i) {
    affine_equality(0, i + index_offset) = power_t[i];
    affine_equality(1, i + spline_order_ + index_offset) = power_t[i];
  }
  return add_equality_constraint(affine_equality, affine_boundary);
}

bool Spline2dConstraint::add_point_angle_constraint(const double t,
                                                    const double angle) {
  const std::size_t index = find_index(t);
  const std::size_t index_offset = index * 2 * spline_order_;
  const double rel_t = t - t_knots_[index];

  // add equality constraint
  Eigen::MatrixXd affine_equality = Eigen::MatrixXd::Zero(1, total_param_);
  Eigen::MatrixXd affine_boundary = Eigen::MatrixXd::Zero(1, 1);
  std::vector<double> line_derivative_coef =
      affine_derivative_coef(angle, rel_t);
  for (std::size_t i = 0; i < line_derivative_coef.size(); ++i) {
    affine_equality(0, i + index_offset) = line_derivative_coef[i];
  }

  // add inequality constraint
  Eigen::MatrixXd affine_inequality = Eigen::MatrixXd::Zero(2, total_param_);
  Eigen::MatrixXd affine_inequality_boundary = Eigen::MatrixXd::Zero(2, 1);
  std::vector<double> t_coef = derivative_coef(rel_t);
  int x_sign = 1;
  int y_sign = 1;
  double normalized_angle = fmod(angle, M_PI * 2);
  if (normalized_angle < 0) {
    normalized_angle += M_PI * 2;
  }
  if (normalized_angle > (M_PI / 2) && normalized_angle < (M_PI * 1.5)) {
    x_sign = -1;
    affine_inequality_boundary(0, 0) *= -1;
  }

  if (normalized_angle >= M_PI) {
    y_sign = -1;
    affine_inequality_boundary(1, 0) *= -1;
  }

  for (std::size_t i = 0; i < t_coef.size(); ++i) {
    affine_inequality(0, i + index_offset) = t_coef[i] * x_sign;
    affine_inequality(1, i + index_offset + spline_order_) = t_coef[i] * y_sign;
  }
  if (!add_equality_constraint(affine_equality, affine_boundary)) {
    return false;
  }
  return add_inequality_constraint(affine_inequality,
                                   affine_inequality_boundary);
}

// guarantee upto values are joint
bool Spline2dConstraint::add_fx_smooth_constraint() {
  if (t_knots_.size() < 3) {
    return false;
  }
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(2 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(2 * (t_knots_.size() - 2), 1);
  for (std::size_t i = 0; i + 2 < t_knots_.size(); ++i) {
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const std::size_t index_offset = 2 * i * spline_order_;
    std::vector<double> power_t = poly_coef(rel_t);

    for (std::size_t j = 0; j < spline_order_; ++j) {
      affine_equality(2 * i, j + index_offset) = power_t[j];
      affine_equality(2 * i + 1, j + index_offset + spline_order_) = power_t[j];
    }
    affine_equality(2 * i, index_offset + 2 * spline_order_) = -1.0;
    affine_equality(2 * i + 1, index_offset + 3 * spline_order_) = -1.0;
  }
  return add_equality_constraint(affine_equality, affine_boundary);
}

// guarantee upto derivative are joint
bool Spline2dConstraint::add_derivative_smooth_constraint() {
  if (t_knots_.size() < 3) {
    return false;
  }
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(4 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(4 * (t_knots_.size() - 2), 1);

  for (std::size_t i = 0; i + 2 < t_knots_.size(); ++i) {
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const std::size_t index_offset = 2 * i * spline_order_;
    std::vector<double> power_t = poly_coef(rel_t);
    std::vector<double> derivative_t = derivative_coef(rel_t);
    for (std::size_t j = 0; j < spline_order_; ++j) {
      affine_equality(4 * i, j + index_offset) = power_t[j];
      affine_equality(4 * i + 1, j + index_offset) = derivative_t[j];
      affine_equality(4 * i + 2, j + index_offset + spline_order_) = power_t[j];
      affine_equality(4 * i + 3, j + index_offset + spline_order_) =
          derivative_t[j];
    }
    affine_equality(4 * i, index_offset + 2 * spline_order_) = -1.0;
    affine_equality(4 * i + 1, index_offset + 2 * spline_order_ + 1) = -1.0;
    affine_equality(4 * i + 2, index_offset + 3 * spline_order_) = -1.0;
    affine_equality(4 * i + 3, index_offset + 3 * spline_order_ + 1) = -1.0;
  }
  return add_equality_constraint(affine_equality, affine_boundary);
}

// guarantee upto second order derivative are joint
bool Spline2dConstraint::add_second_derivative_smooth_constraint() {
  if (t_knots_.size() < 3) {
    return false;
  }
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(6 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(6 * (t_knots_.size() - 2), 1);

  for (std::size_t i = 0; i + 2 < t_knots_.size(); ++i) {
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const std::size_t index_offset = 2 * i * spline_order_;
    std::vector<double> power_t = poly_coef(rel_t);
    std::vector<double> derivative_t = derivative_coef(rel_t);
    std::vector<double> second_derivative_t = second_derivative_coef(rel_t);
    for (std::size_t j = 0; j < spline_order_; ++j) {
      affine_equality(6 * i, j + index_offset) = power_t[j];
      affine_equality(6 * i + 1, j + index_offset) = derivative_t[j];
      affine_equality(6 * i + 2, j + index_offset) = second_derivative_t[j];
      affine_equality(6 * i + 3, j + index_offset + spline_order_) = power_t[j];
      affine_equality(6 * i + 4, j + index_offset + spline_order_) =
          derivative_t[j];
      affine_equality(6 * i + 5, j + index_offset + spline_order_) =
          second_derivative_t[j];
    }
    affine_equality(6 * i, index_offset + 2 * spline_order_) = -1.0;
    affine_equality(6 * i + 1, index_offset + 2 * spline_order_ + 1) = -1.0;
    affine_equality(6 * i + 2, index_offset + 2 * spline_order_ + 2) = -2.0;
    affine_equality(6 * i + 3, index_offset + 3 * spline_order_) = -1.0;
    affine_equality(6 * i + 4, index_offset + 3 * spline_order_ + 1) = -1.0;
    affine_equality(6 * i + 5, index_offset + 3 * spline_order_ + 2) = -2.0;
  }
  return add_equality_constraint(affine_equality, affine_boundary);
}

// guarantee upto third order derivative are joint
bool Spline2dConstraint::add_third_derivative_smooth_constraint() {
  if (t_knots_.size() < 3) {
    return false;
  }
  Eigen::MatrixXd affine_equality =
      Eigen::MatrixXd::Zero(8 * (t_knots_.size() - 2), total_param_);
  Eigen::MatrixXd affine_boundary =
      Eigen::MatrixXd::Zero(8 * (t_knots_.size() - 2), 1);

  for (std::size_t i = 0; i + 2 < t_knots_.size(); ++i) {
    const double rel_t = t_knots_[i + 1] - t_knots_[i];
    const std::size_t index_offset = 2 * i * spline_order_;
    std::vector<double> power_t = poly_coef(rel_t);
    std::vector<double> derivative_t = derivative_coef(rel_t);
    std::vector<double> second_derivative_t = second_derivative_coef(rel_t);
    std::vector<double> third_derivative_t = third_derivative_coef(rel_t);
    for (std::size_t j = 0; j < spline_order_; ++j) {
      affine_equality(8 * i, j + index_offset) = power_t[j];
      affine_equality(8 * i + 1, j + index_offset) = derivative_t[j];
      affine_equality(8 * i + 2, j + index_offset) = second_derivative_t[j];
      affine_equality(8 * i + 3, j + index_offset) = third_derivative_t[j];
      affine_equality(8 * i + 4, j + index_offset + spline_order_) = power_t[j];
      affine_equality(8 * i + 5, j + index_offset + spline_order_) =
          derivative_t[j];
      affine_equality(8 * i + 6, j + index_offset + spline_order_) =
          second_derivative_t[j];
      affine_equality(8 * i + 7, j + index_offset + spline_order_) =
          third_derivative_t[j];
    }
    affine_equality(8 * i, index_offset + 2 * spline_order_) = -1.0;
    affine_equality(8 * i + 1, index_offset + 2 * spline_order_ + 1) = -1.0;
    affine_equality(8 * i + 2, index_offset + 2 * spline_order_ + 2) = -2.0;
    affine_equality(8 * i + 3, index_offset + 2 * spline_order_ + 3) = -6.0;
    affine_equality(8 * i + 4, index_offset + 3 * spline_order_) = -1.0;
    affine_equality(8 * i + 5, index_offset + 3 * spline_order_ + 1) = -1.0;
    affine_equality(8 * i + 6, index_offset + 3 * spline_order_ + 2) = -2.0;
    affine_equality(8 * i + 7, index_offset + 3 * spline_order_ + 3) = -6.0;
  }
  return add_equality_constraint(affine_equality, affine_boundary);
}

/**
*   @brief: output interface inequality constraint
**/
const AffineConstraint& Spline2dConstraint::inequality_constraint() const {
  return inequality_constraint_;
}

const AffineConstraint& Spline2dConstraint::equality_constraint() const {
  return equality_constraint_;
}

std::size_t Spline2dConstraint::find_index(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin() + 1, t_knots_.end(), t);
  return std::min(t_knots_.size() - 1,
                  static_cast<std::size_t>(upper_bound - t_knots_.begin())) -
         1;
}

std::vector<double> Spline2dConstraint::affine_coef(const double angle,
                                                    const double t) const {
  std::vector<double> result(spline_order_ * 2, 0.0);
  double x_coef = -std::sin(angle);
  double y_coef = std::cos(angle);
  for (std::size_t i = 0; i < spline_order_; ++i) {
    result[i] = x_coef;
    result[i + spline_order_] = y_coef;
    x_coef *= t;
    y_coef *= t;
  }
  return result;
}

std::vector<double> Spline2dConstraint::affine_derivative_coef(
    const double angle, const double t) const {
  std::vector<double> result(spline_order_ * 2, 0.0);
  double x_coef = -std::sin(angle);
  double y_coef = std::cos(angle);
  std::vector<double> power_t = poly_coef(t);
  for (std::size_t i = 1; i < spline_order_; ++i) {
    result[i] = x_coef * power_t[i - 1] * i;
    result[i + spline_order_] = y_coef * power_t[i - 1] * i;
  }
  return result;
}

std::vector<double> Spline2dConstraint::affine_second_derivative_coef(
    const double angle, const double t) const {
  std::vector<double> result(spline_order_ * 2, 0.0);
  double x_coef = -std::sin(angle);
  double y_coef = std::cos(angle);
  std::vector<double> power_t = poly_coef(t);
  for (std::size_t i = 2; i < spline_order_; ++i) {
    result[i] = x_coef * power_t[i - 2] * i * (i - 1);
    result[i + spline_order_] = y_coef * power_t[i - 2] * i * (i - 1);
  }
  return result;
}

std::vector<double> Spline2dConstraint::affine_third_derivative_coef(
    const double angle, const double t) const {
  std::vector<double> result(spline_order_ * 2, 0.0);
  double x_coef = -std::sin(angle);
  double y_coef = std::cos(angle);
  std::vector<double> power_t = poly_coef(t);
  for (std::size_t i = 3; i < spline_order_; ++i) {
    result[i] = x_coef * power_t[i - 3] * i * (i - 1) * (i - 2);
    result[i + spline_order_] = y_coef * power_t[i - 3] * i * (i - 1) * (i - 2);
  }
  return result;
}

double Spline2dConstraint::sign_distance(const Vec2d& xy_point,
                                         const double angle) const {
  return -std::sin(angle) * xy_point.x() + std::cos(angle) * xy_point.y();
}

std::vector<double> Spline2dConstraint::poly_coef(const double t) const {
  std::vector<double> result(spline_order_, 1.0);
  for (std::size_t i = 1; i < result.size(); ++i) {
    result[i] = result[i - 1] * t;
  }
  return result;
}

std::vector<double> Spline2dConstraint::derivative_coef(const double t) const {
  std::vector<double> result(spline_order_, 0.0);
  std::vector<double> power_t = poly_coef(t);
  for (std::size_t i = 1; i < result.size(); ++i) {
    result[i] = power_t[i - 1] * i;
  }
  return result;
}

std::vector<double> Spline2dConstraint::second_derivative_coef(
    const double t) const {
  std::vector<double> result(spline_order_, 0.0);
  std::vector<double> power_t = poly_coef(t);
  for (std::size_t i = 2; i < result.size(); ++i) {
    result[i] = power_t[i - 2] * i * (i - 1);
  }
  return result;
}

std::vector<double> Spline2dConstraint::third_derivative_coef(
    const double t) const {
  std::vector<double> result(spline_order_, 0.0);
  std::vector<double> power_t = poly_coef(t);
  for (std::size_t i = 3; i < result.size(); ++i) {
    result[i] = power_t[i - 3] * i * (i - 1) * (i - 2);
  }
  return result;
}

}  // namespace planning
}  // namespace apollo
