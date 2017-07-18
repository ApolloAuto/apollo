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
 * @file : spline_1d_constraint.cc
 * @brief: wrapp up solver constraint interface with direct methods and preset
 *methods
 **/

#include "modules/planning/math/smoothing_spline/spline_1d_constraint.h"

namespace apollo {
namespace planning {

Spline1dConstraint::Spline1dConstraint(const Spline1d& pss)
    : x_knots_(pss.x_knots()), spline_order_(pss.spline_order()) {
  inequality_constraint_.set_is_equality(false);
  equality_constraint_.set_is_equality(true);
}

Spline1dConstraint::Spline1dConstraint(const std::vector<double>& x_knots,
                                       const std::size_t spline_order)
    : x_knots_(x_knots), spline_order_(spline_order) {
  inequality_constraint_.set_is_equality(false);
  equality_constraint_.set_is_equality(true);
}

bool Spline1dConstraint::add_inequality_constraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  return inequality_constraint_.add_constraint(constraint_matrix,
                                               constraint_boundary);
}

bool Spline1dConstraint::add_equality_constraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  return equality_constraint_.add_constraint(constraint_matrix,
                                             constraint_boundary);
}

bool Spline1dConstraint::add_fx_boundary(
    const std::vector<double>& x_coord, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  std::vector<double> filtered_lower_bound;
  std::vector<double> filtered_upper_bound;
  std::vector<double> filtered_lower_bound_x;
  std::vector<double> filtered_upper_bound_x;

  if (x_knots_.size() < 2) {
    return false;
  }

  if (!filter_constraints(x_coord, lower_bound, upper_bound,
                          &filtered_lower_bound_x, &filtered_lower_bound,
                          &filtered_upper_bound_x, &filtered_upper_bound)) {
    return false;
  };
  // emplace affine constraints
  Eigen::MatrixXd inequality_constraint = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(),
      (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd inequality_boundary = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(), 1);

  for (std::size_t i = 0; i < filtered_lower_bound.size(); ++i) {
    std::size_t index = find_index(filtered_lower_bound_x[i]);

    const double corrected_x = filtered_lower_bound_x[i] - x_knots_[index];
    double coef = 1.0;
    for (std::size_t j = 0; j < spline_order_; ++j) {
      inequality_constraint(i, j + index * spline_order_) = coef;
      coef *= corrected_x;
    }
    inequality_boundary(i, 0) = filtered_lower_bound[i];
  }

  for (std::size_t i = 0; i < filtered_upper_bound.size(); ++i) {
    std::size_t index = find_index(filtered_upper_bound_x[i]);
    const double corrected_x = filtered_upper_bound_x[i] - x_knots_[index];
    double coef = -1.0;
    for (std::size_t j = 0; j < spline_order_; ++j) {
      inequality_constraint(i + filtered_upper_bound.size(),
                            j + index * spline_order_) = coef;
      coef *= corrected_x;
    }
    inequality_boundary(i + filtered_upper_bound.size(), 0) =
        -filtered_upper_bound[i];
  }

  return inequality_constraint_.add_constraint(inequality_constraint,
                                               inequality_boundary);
}

bool Spline1dConstraint::add_derivative_boundary(
    const std::vector<double>& x_coord, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  std::vector<double> filtered_lower_bound;
  std::vector<double> filtered_upper_bound;
  std::vector<double> filtered_lower_bound_x;
  std::vector<double> filtered_upper_bound_x;

  if (x_knots_.size() < 2) {
    return false;
  }

  if (!filter_constraints(x_coord, lower_bound, upper_bound,
                          &filtered_lower_bound_x, &filtered_lower_bound,
                          &filtered_upper_bound_x, &filtered_upper_bound)) {
    return false;
  };

  // emplace affine constraints
  Eigen::MatrixXd inequality_constraint = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(),
      (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd inequality_boundary = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(), 1);

  for (std::size_t i = 0; i < filtered_lower_bound.size(); ++i) {
    std::size_t index = find_index(filtered_lower_bound_x[i]);
    const double corrected_x = filtered_lower_bound_x[i] - x_knots_[index];
    double coef = 1.0;
    for (std::size_t j = 1; j < spline_order_; ++j) {
      inequality_constraint(i, j + index * spline_order_) = coef * j;
      coef *= corrected_x;
    }
    inequality_boundary(i, 0) = filtered_lower_bound[i];
  }

  for (std::size_t i = 0; i < filtered_upper_bound.size(); ++i) {
    std::size_t index = find_index(filtered_upper_bound_x[i]);
    const double corrected_x = filtered_upper_bound_x[i] - x_knots_[index];
    double coef = -1.0;
    for (std::size_t j = 1; j < spline_order_; ++j) {
      inequality_constraint(i + filtered_upper_bound.size(),
                            j + index * spline_order_) = coef * j;
      coef *= corrected_x;
    }
    inequality_boundary(i + filtered_upper_bound.size(), 0) =
        -filtered_upper_bound[i];
  }
  return inequality_constraint_.add_constraint(inequality_constraint,
                                               inequality_boundary);
}

bool Spline1dConstraint::add_second_derivative_boundary(
    const std::vector<double>& x_coord, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  std::vector<double> filtered_lower_bound;
  std::vector<double> filtered_upper_bound;
  std::vector<double> filtered_lower_bound_x;
  std::vector<double> filtered_upper_bound_x;

  if (x_knots_.size() < 2) {
    return false;
  }

  if (!filter_constraints(x_coord, lower_bound, upper_bound,
                          &filtered_lower_bound_x, &filtered_lower_bound,
                          &filtered_upper_bound_x, &filtered_upper_bound)) {
    return false;
  };

  // emplace affine constraints
  Eigen::MatrixXd inequality_constraint = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(),
      (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd inequality_boundary = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(), 1);

  for (std::size_t i = 0; i < filtered_lower_bound.size(); ++i) {
    std::size_t index = find_index(filtered_lower_bound_x[i]);
    const double corrected_x = filtered_lower_bound_x[i] - x_knots_[index];
    double coef = 1.0;
    for (std::size_t j = 2; j < spline_order_; ++j) {
      inequality_constraint(i, j + index * spline_order_) = coef * j * (j - 1);
      coef *= corrected_x;
    }
    inequality_boundary(i, 0) = filtered_lower_bound[i];
  }

  for (std::size_t i = 0; i < filtered_upper_bound.size(); ++i) {
    std::size_t index = find_index(filtered_upper_bound_x[i]);
    const double corrected_x = filtered_upper_bound_x[i] - x_knots_[index];
    double coef = -1.0;
    for (std::size_t j = 2; j < spline_order_; ++j) {
      inequality_constraint(i + filtered_upper_bound.size(),
                            j + index * spline_order_) = coef * j * (j - 1);
      coef *= corrected_x;
    }
    inequality_boundary(i + filtered_upper_bound.size(), 0) =
        -filtered_upper_bound[i];
  }
  return inequality_constraint_.add_constraint(inequality_constraint,
                                               inequality_boundary);
}

bool Spline1dConstraint::add_third_derivative_boundary(
    const std::vector<double>& x_coord, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  std::vector<double> filtered_lower_bound;
  std::vector<double> filtered_upper_bound;
  std::vector<double> filtered_lower_bound_x;
  std::vector<double> filtered_upper_bound_x;

  if (!filter_constraints(x_coord, lower_bound, upper_bound,
                          &filtered_lower_bound_x, &filtered_lower_bound,
                          &filtered_upper_bound_x, &filtered_upper_bound)) {
    return false;
  };

  if (x_knots_.size() < 2) {
    return false;
  }

  // emplace affine constraints
  Eigen::MatrixXd inequality_constraint = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(),
      (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd inequality_boundary = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(), 1);

  for (std::size_t i = 0; i < filtered_lower_bound.size(); ++i) {
    std::size_t index = find_index(filtered_lower_bound_x[i]);
    const double corrected_x = filtered_lower_bound_x[i] - x_knots_[index];
    double coef = 1.0;
    for (std::size_t j = 3; j < spline_order_; ++j) {
      inequality_constraint(i, j + index * spline_order_) =
          coef * j * (j - 1) * (j - 2);
      coef *= corrected_x;
    }
    inequality_boundary(i, 0) = filtered_lower_bound[i];
  }

  for (std::size_t i = 0; i < filtered_upper_bound.size(); ++i) {
    std::size_t index = find_index(filtered_upper_bound_x[i]);
    const double corrected_x = filtered_upper_bound_x[i] - x_knots_[index];
    double coef = -1.0;
    for (std::size_t j = 3; j < spline_order_; ++j) {
      inequality_constraint(i + filtered_upper_bound.size(),
                            j + index * spline_order_) =
          coef * j * (j - 1) * (j - 2);
      coef *= corrected_x;
    }
    inequality_boundary(i + filtered_upper_bound.size(), 0) =
        -filtered_upper_bound[i];
  }
  return inequality_constraint_.add_constraint(inequality_constraint,
                                               inequality_boundary);
}

bool Spline1dConstraint::add_point_fx_constraint(const double x,
                                                 const double fx) {
  std::size_t index = find_index(x);
  std::vector<double> power_x;
  generate_power_x(x - x_knots_[index], spline_order_, &power_x);
  Eigen::MatrixXd equality_constraint =
      Eigen::MatrixXd::Zero(1, (x_knots_.size() - 1) * spline_order_);
  std::size_t index_offset = index * spline_order_;
  for (std::size_t i = 0; i < spline_order_; ++i) {
    equality_constraint(0, index_offset + i) = power_x[i];
  }
  Eigen::MatrixXd equality_boundary(1, 1);
  equality_boundary(0, 0) = fx;
  return add_equality_constraint(equality_constraint, equality_boundary);
}

bool Spline1dConstraint::add_point_derivative_constraint(const double x,
                                                         const double dfx) {
  std::size_t index = find_index(x);
  std::vector<double> power_x;
  generate_power_x(x - x_knots_[index], spline_order_, &power_x);
  Eigen::MatrixXd equality_constraint =
      Eigen::MatrixXd::Zero(1, (x_knots_.size() - 1) * spline_order_);
  std::size_t index_offset = index * spline_order_;
  for (std::size_t i = 1; i < spline_order_; ++i) {
    equality_constraint(0, index_offset + i) = power_x[i - 1] * i;
  }
  Eigen::MatrixXd equality_boundary(1, 1);
  equality_boundary(0, 0) = dfx;
  return add_equality_constraint(equality_constraint, equality_boundary);
}

bool Spline1dConstraint::add_point_second_derivative_constraint(
    const double x, const double ddfx) {
  std::size_t index = find_index(x);
  std::vector<double> power_x;
  generate_power_x(x - x_knots_[index], spline_order_, &power_x);
  Eigen::MatrixXd equality_constraint =
      Eigen::MatrixXd::Zero(1, (x_knots_.size() - 1) * spline_order_);
  std::size_t index_offset = index * spline_order_;
  for (std::size_t i = 2; i < spline_order_; ++i) {
    equality_constraint(0, index_offset + i) = power_x[i - 2] * i * (i - 1);
  }
  Eigen::MatrixXd equality_boundary(1, 1);
  equality_boundary(0, 0) = ddfx;
  return add_equality_constraint(equality_constraint, equality_boundary);
}

bool Spline1dConstraint::add_point_third_derivative_constraint(
    const double x, const double dddfx) {
  std::size_t index = find_index(x);
  std::vector<double> power_x;
  generate_power_x(x - x_knots_[index], spline_order_, &power_x);
  Eigen::MatrixXd equality_constraint =
      Eigen::MatrixXd::Zero(1, (x_knots_.size() - 1) * spline_order_);
  std::size_t index_offset = index * spline_order_;
  for (std::size_t i = 3; i < spline_order_; ++i) {
    equality_constraint(0, index_offset + i) =
        power_x[i - 3] * i * (i - 1) * (i - 2);
  }
  Eigen::MatrixXd equality_boundary(1, 1);
  equality_boundary(0, 0) = dddfx;
  return add_equality_constraint(equality_constraint, equality_boundary);
}

bool Spline1dConstraint::add_fx_smooth_constraint() {
  if (x_knots_.size() < 3) {
    return false;
  }
  Eigen::MatrixXd equality_constraint = Eigen::MatrixXd::Zero(
      x_knots_.size() - 2, (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd equality_boundary =
      Eigen::MatrixXd::Zero(x_knots_.size() - 2, 1);

  for (std::size_t i = 0; i < x_knots_.size() - 2; ++i) {
    double left_coef = 1.0;
    double right_coef = -1.0;
    const double x_left = x_knots_[i + 1] - x_knots_[i];
    const double x_right = 0.0;
    for (std::size_t j = 0; j < spline_order_; ++j) {
      equality_constraint(i, spline_order_ * i + j) = left_coef;
      equality_constraint(i, spline_order_ * (i + 1) + j) = right_coef;
      left_coef *= x_left;
      right_coef *= x_right;
    }
  }
  return equality_constraint_.add_constraint(equality_constraint,
                                             equality_boundary);
}

bool Spline1dConstraint::add_derivative_smooth_constraint() {
  if (x_knots_.size() < 3) {
    return false;
  }

  const std::size_t n_constraint = (x_knots_.size() - 2) * 2;
  Eigen::MatrixXd equality_constraint = Eigen::MatrixXd::Zero(
      n_constraint, (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd equality_boundary = Eigen::MatrixXd::Zero(n_constraint, 1);

  for (std::size_t i = 0; i < n_constraint; i += 2) {
    double left_coef = 1.0;
    double right_coef = -1.0;
    double left_dcoef = 1.0;
    double right_dcoef = -1.0;
    const double x_left = x_knots_[i / 2 + 1] - x_knots_[i / 2];
    const double x_right = 0.0;
    for (std::size_t j = 0; j < spline_order_; ++j) {
      equality_constraint(i, spline_order_ * (i / 2) + j) = left_coef;
      equality_constraint(i, spline_order_ * ((i / 2) + 1) + j) = right_coef;
      if (j >= 1) {
        equality_constraint(i + 1, spline_order_ * (i / 2) + j) =
            left_dcoef * j;
        equality_constraint(i + 1, spline_order_ * ((i / 2) + 1) + j) =
            right_dcoef * j;
        left_dcoef = left_coef;
        right_dcoef = right_coef;
      }
      left_coef *= x_left;
      right_coef *= x_right;
    }
  }
  return equality_constraint_.add_constraint(equality_constraint,
                                             equality_boundary);
}

bool Spline1dConstraint::add_second_derivative_smooth_constraint() {
  if (x_knots_.size() < 3) {
    return false;
  }

  const std::size_t n_constraint = (x_knots_.size() - 2) * 3;
  Eigen::MatrixXd equality_constraint = Eigen::MatrixXd::Zero(
      n_constraint, (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd equality_boundary = Eigen::MatrixXd::Zero(n_constraint, 1);

  for (std::size_t i = 0; i < n_constraint; i += 3) {
    double left_coef = 1.0;
    double right_coef = -1.0;
    double left_dcoef = 1.0;
    double right_dcoef = -1.0;
    double left_ddcoef = 1.0;
    double right_ddcoef = -1.0;

    const double x_left = x_knots_[i / 3 + 1] - x_knots_[i / 3];
    const double x_right = 0.0;
    for (std::size_t j = 0; j < spline_order_; ++j) {
      equality_constraint(i, spline_order_ * (i / 3) + j) = left_coef;
      equality_constraint(i, spline_order_ * (i / 3 + 1) + j) = right_coef;

      if (j >= 2) {
        equality_constraint(i + 2, spline_order_ * i / 3 + j) =
            left_ddcoef * j * (j - 1);
        equality_constraint(i + 2, spline_order_ * (i / 3 + 1) + j) =
            right_ddcoef * j * (j - 1);
        left_ddcoef = left_dcoef;
        right_ddcoef = right_dcoef;
      }

      if (j >= 1) {
        equality_constraint(i + 1, spline_order_ * (i / 3) + j) =
            left_dcoef * j;
        equality_constraint(i + 1, spline_order_ * (i / 3 + 1) + j) =
            right_dcoef * j;
        left_dcoef = left_coef;
        right_dcoef = right_coef;
      }
      left_coef *= x_left;
      right_coef *= x_right;
    }
  }
  return equality_constraint_.add_constraint(equality_constraint,
                                             equality_boundary);
}

bool Spline1dConstraint::add_third_derivative_smooth_constraint() {
  if (x_knots_.size() < 3) {
    return false;
  }

  const std::size_t n_constraint = (x_knots_.size() - 2) * 4;
  Eigen::MatrixXd equality_constraint = Eigen::MatrixXd::Zero(
      n_constraint, (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd equality_boundary = Eigen::MatrixXd::Zero(n_constraint, 1);

  for (std::size_t i = 0; i < n_constraint; i += 4) {
    double left_coef = 1.0;
    double right_coef = -1.0;
    double left_dcoef = 1.0;
    double right_dcoef = -1.0;
    double left_ddcoef = 1.0;
    double right_ddcoef = -1.0;
    double left_dddcoef = 1.0;
    double right_dddcoef = -1.0;

    const double x_left = x_knots_[i / 4 + 1] - x_knots_[i / 4];
    const double x_right = 0.0;
    for (std::size_t j = 0; j < spline_order_; ++j) {
      equality_constraint(i, spline_order_ * i / 4 + j) = left_coef;
      equality_constraint(i, spline_order_ * (i / 4 + 1) + j) = right_coef;

      if (j >= 3) {
        equality_constraint(i + 3, spline_order_ * i / 4 + j) =
            left_dddcoef * j * (j - 1) * (j - 2);
        equality_constraint(i + 3, spline_order_ * (i / 4 + 1) + j) =
            right_dddcoef * j * (j - 1) * (j - 2);
        left_dddcoef = left_ddcoef;
        right_dddcoef = right_ddcoef;
      }

      if (j >= 2) {
        equality_constraint(i + 2, spline_order_ * i / 4 + j) =
            left_ddcoef * j * (j - 1);
        equality_constraint(i + 2, spline_order_ * (i / 4 + 1) + j) =
            right_ddcoef * j * (j - 1);
        left_ddcoef = left_dcoef;
        right_ddcoef = right_dcoef;
      }

      if (j >= 1) {
        equality_constraint(i + 1, spline_order_ * i / 4 + j) = left_dcoef * j;
        equality_constraint(i + 1, spline_order_ * (i / 4 + 1) + j) =
            right_dcoef * j;
        left_dcoef = left_coef;
        right_dcoef = right_coef;
      }

      left_coef *= x_left;
      right_coef *= x_right;
    }
  }
  return equality_constraint_.add_constraint(equality_constraint,
                                             equality_boundary);
}

bool Spline1dConstraint::add_monotone_fx_inequality_constraint(
    const std::vector<double>& x_coord) {
  if (x_coord.size() < 2) {
    // no inequality constraint needed
    return false;
  }

  Eigen::MatrixXd inequality_constraint = Eigen::MatrixXd::Zero(
      x_coord.size() - 1, (x_knots_.size() - 1) * spline_order_);
  Eigen::MatrixXd inequality_boundary =
      Eigen::MatrixXd::Zero(x_coord.size() - 1, 1);

  std::size_t prev_spline_index = find_index(x_coord[0]);
  double prev_rel_x = x_coord[0] - x_knots_[prev_spline_index];
  std::vector<double> prev_coef;
  generate_power_x(prev_rel_x, spline_order_, &prev_coef);
  for (std::size_t i = 1; i < x_coord.size(); ++i) {
    std::size_t cur_spline_index = find_index(x_coord[i]);
    double cur_rel_x = x_coord[i] - x_knots_[cur_spline_index];
    std::vector<double> cur_coef;
    generate_power_x(cur_rel_x, spline_order_, &cur_coef);
    // if constraint on the same spline
    if (cur_spline_index == prev_spline_index) {
      for (std::size_t j = 0; j < cur_coef.size(); ++j) {
        inequality_constraint(i - 1, cur_spline_index * spline_order_ + j) =
            cur_coef[j] - prev_coef[j];
      }
    } else {
      // if not on the same spline
      for (std::size_t j = 0; j < cur_coef.size(); ++j) {
        inequality_constraint(i - 1, prev_spline_index * spline_order_ + j) =
            -prev_coef[j];
        inequality_constraint(i - 1, cur_spline_index * spline_order_ + j) =
            cur_coef[j];
      }
    }
    prev_coef = cur_coef;
  }

  return inequality_constraint_.add_constraint(inequality_constraint,
                                               inequality_boundary);
}

bool Spline1dConstraint::add_monotone_fx_inequality_constraint_at_knots() {
  return add_monotone_fx_inequality_constraint(x_knots_);
}

const AffineConstraint& Spline1dConstraint::inequality_constraint() const {
  return inequality_constraint_;
}

const AffineConstraint& Spline1dConstraint::equality_constraint() const {
  return equality_constraint_;
}

std::size_t Spline1dConstraint::find_index(const double x) const {
  auto upper_bound = std::upper_bound(x_knots_.begin() + 1, x_knots_.end(), x);
  return std::min(x_knots_.size() - 1,
                  static_cast<std::size_t>(upper_bound - x_knots_.begin())) -
         1;
}

bool Spline1dConstraint::filter_constraints(
    const std::vector<double>& x_coord, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound,
    std::vector<double>* const filtered_lower_bound_x,
    std::vector<double>* const filtered_lower_bound,
    std::vector<double>* const filtered_upper_bound_x,
    std::vector<double>* const filtered_upper_bound) {
  filtered_lower_bound->clear();
  filtered_upper_bound->clear();
  filtered_lower_bound_x->clear();
  filtered_upper_bound_x->clear();

  const double inf = std::numeric_limits<double>::infinity();

  filtered_lower_bound->reserve(lower_bound.size());
  filtered_lower_bound_x->reserve(lower_bound.size());

  filtered_upper_bound->reserve(upper_bound.size());
  filtered_upper_bound_x->reserve(upper_bound.size());

  for (std::size_t i = 0; i < lower_bound.size(); ++i) {
    if (std::isnan(lower_bound[i]) || lower_bound[i] == inf) {
      return false;
    }
    if (lower_bound[i] < inf && lower_bound[i] > -inf) {
      filtered_lower_bound->emplace_back(lower_bound[i]);
      filtered_lower_bound_x->emplace_back(x_coord[i]);
    }
  }

  for (std::size_t i = 0; i < lower_bound.size(); ++i) {
    if (std::isnan(upper_bound[i]) || upper_bound[i] == -inf) {
      return false;
    }
    if (upper_bound[i] < inf && upper_bound[i] > -inf) {
      filtered_upper_bound->emplace_back(upper_bound[i]);
      filtered_upper_bound_x->emplace_back(x_coord[i]);
    }
  }
  return true;
}

void Spline1dConstraint::generate_power_x(
    const double x, const std::size_t order,
    std::vector<double>* const power_x) const {
  double cur_x = 1;
  for (std::size_t i = 0; i < order; ++i) {
    power_x->emplace_back(cur_x);
    cur_x *= x;
  }
}

}  // namespace planning
}  // namespace apollo
