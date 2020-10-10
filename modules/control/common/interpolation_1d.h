/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "unsupported/Eigen/Splines"

namespace apollo {
namespace control {

class Interpolation1D {
 public:
  typedef std::vector<std::pair<double, double>> DataType;

  Interpolation1D() = default;

  // Return true if init is ok.
  bool Init(const DataType& xy);

  // Only interpolate x between [x_min, x_max]
  // For x out of range, start or end y value is returned.
  double Interpolate(double x) const;

 private:
  // Helpers to scale X values down to [0, 1]
  double ScaledValue(double x) const;

  Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const& x_vec) const;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_start_ = 0.0;
  double y_end_ = 0.0;

  // Spline of one-dimensional "points."
  std::unique_ptr<Eigen::Spline<double, 1>> spline_;
};

}  // namespace control
}  // namespace apollo
