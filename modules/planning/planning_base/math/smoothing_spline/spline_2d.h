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
 * @file : spline_2d.h
 * @brief: piecewise smoothing spline 2d class
 **/

#pragma once

#include <utility>
#include <vector>

#include "Eigen/Core"

#include "modules/planning/planning_base/math/polynomial_xd.h"
#include "modules/planning/planning_base/math/smoothing_spline/spline_2d_seg.h"

namespace apollo {
namespace planning {

class Spline2d {
 public:
  Spline2d(const std::vector<double>& t_knots, const uint32_t order);
  std::pair<double, double> operator()(const double t) const;
  double x(const double t) const;
  double y(const double t) const;
  double DerivativeX(const double t) const;
  double DerivativeY(const double t) const;
  double SecondDerivativeX(const double t) const;
  double SecondDerivativeY(const double t) const;
  double ThirdDerivativeX(const double t) const;
  double ThirdDerivativeY(const double t) const;
  bool set_splines(const Eigen::MatrixXd& params, const uint32_t order);
  const Spline2dSeg& smoothing_spline(const uint32_t index) const;
  const std::vector<double>& t_knots() const;
  uint32_t spline_order() const;

 private:
  uint32_t find_index(const double x) const;

 private:
  std::vector<Spline2dSeg> splines_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
};

}  // namespace planning
}  // namespace apollo
