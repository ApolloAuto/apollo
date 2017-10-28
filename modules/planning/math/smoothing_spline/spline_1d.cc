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

#include "modules/planning/math/smoothing_spline/spline_1d.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace apollo {
namespace planning {

Spline1d::Spline1d(const std::vector<double>& x_knots,
                   const uint32_t order)
    : x_knots_(x_knots), spline_order_(order) {
  if (x_knots.size() > 1) {
    for (uint32_t i = 1; i < x_knots_.size(); ++i) {
      splines_.emplace_back(spline_order_);
    }
  }
}

double Spline1d::operator()(const double x) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index](x - x_knots_[index]);
}

double Spline1d::Derivative(const double x) const {
  // zero order spline
  if (splines_.size() == 0) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index].Derivative(x - x_knots_[index]);
}

double Spline1d::SecondOrderDerivative(const double x) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index].SecondOrderDerivative(x - x_knots_[index]);
}

double Spline1d::ThirdOrderDerivative(const double x) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  uint32_t index = FindIndex(x);
  return splines_[index].ThirdOrderDerivative(x - x_knots_[index]);
}

bool Spline1d::SetSplineSegs(const Eigen::MatrixXd& params,
                             const uint32_t order) {
  // check if the parameter size fit
  if (x_knots_.size() * order !=
      order + static_cast<uint32_t>(params.rows())) {
    return false;
  }
  for (uint32_t i = 0; i < splines_.size(); ++i) {
    std::vector<double> spline_piece(order, 0.0);
    for (uint32_t j = 0; j < order; ++j) {
      spline_piece[j] = params(i * order + j, 0);
    }
    splines_[i].SetParams(spline_piece);
  }
  spline_order_ = order;
  return true;
}

Spline1dSeg* Spline1d::mutable_smoothing_spline(const uint32_t index) {
  if (index >= splines_.size()) {
    return nullptr;
  } else {
    // TODO(all): the address of spline_[index] may change, so we need a better
    // design here to avoid potential crash.
    return &splines_[index];
  }
}

const std::vector<double>& Spline1d::x_knots() const { return x_knots_; }

uint32_t Spline1d::spline_order() const { return spline_order_; }

uint32_t Spline1d::FindIndex(const double x) const {
  auto upper_bound = std::upper_bound(x_knots_.begin() + 1, x_knots_.end(), x);
  return std::min(static_cast<uint32_t>(x_knots_.size() - 1),
                  static_cast<uint32_t>(upper_bound - x_knots_.begin())) -
         1;
}

}  // namespace planning
}  // namespace apollo
