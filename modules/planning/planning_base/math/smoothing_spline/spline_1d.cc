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
 * @file : piecewise_smoothing_spline.h
 * @brief: polynomial smoothing spline
 **/

#include "modules/planning/planning_base/math/smoothing_spline/spline_1d.h"

namespace apollo {
namespace planning {

Spline1d::Spline1d(const std::vector<double>& x_knots, const uint32_t order)
    : x_knots_(x_knots), spline_order_(order) {
  for (uint32_t i = 1; i < x_knots_.size(); ++i) {
    splines_.emplace_back(spline_order_);
  }
}

double Spline1d::operator()(const double x) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index](x - x_knots_[index]);
}

double Spline1d::Derivative(const double x) const {
  // zero order spline
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index].Derivative(x - x_knots_[index]);
}

double Spline1d::SecondOrderDerivative(const double x) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index].SecondOrderDerivative(x - x_knots_[index]);
}

double Spline1d::ThirdOrderDerivative(const double x) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index].ThirdOrderDerivative(x - x_knots_[index]);
}

bool Spline1d::SetSplineSegs(const Eigen::MatrixXd& param_matrix,
                             const uint32_t order) {
  const uint32_t num_params = order + 1;
  // check if the parameter size fit
  if (x_knots_.size() * num_params !=
      num_params + static_cast<uint32_t>(param_matrix.rows())) {
    return false;
  }
  for (uint32_t i = 0; i < splines_.size(); ++i) {
    std::vector<double> spline_piece(num_params, 0.0);
    for (uint32_t j = 0; j < num_params; ++j) {
      spline_piece[j] = param_matrix(i * num_params + j, 0);
    }
    splines_[i].SetParams(spline_piece);
  }
  spline_order_ = order;
  return true;
}

const std::vector<double>& Spline1d::x_knots() const { return x_knots_; }

uint32_t Spline1d::spline_order() const { return spline_order_; }

const std::vector<Spline1dSeg>& Spline1d::splines() const { return splines_; }

uint32_t Spline1d::FindIndex(const double x) const {
  auto upper_bound = std::upper_bound(x_knots_.begin() + 1, x_knots_.end(), x);
  const uint32_t dis =
      static_cast<uint32_t>(std::distance(x_knots_.begin(), upper_bound));
  if (dis < x_knots_.size()) {
    return dis - 1;
  } else {
    return static_cast<uint32_t>(x_knots_.size()) - 2;
  }
}

}  // namespace planning
}  // namespace apollo
