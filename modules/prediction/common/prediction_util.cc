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

#include "modules/prediction/common/prediction_util.h"

#include <cmath>
#include <limits>

#include "modules/common/log.h"

namespace apollo {
namespace prediction {
namespace util {

using apollo::common::TrajectoryPoint;

double Normalize(const double value, const double mean, const double std) {
  constexpr double eps = 1e-10;
  return (value - mean) / (std + eps);
}

double Sigmoid(const double value) { return 1 / (1 + std::exp(-1.0 * value)); }

double Relu(const double value) { return (value > 0.0) ? value : 0.0; }

int SolveQuadraticEquation(const std::vector<double>& coefficients,
                           std::pair<double, double>* roots) {
  if (coefficients.size() != 3) {
    return -1;
  }
  const double a = coefficients[0];
  const double b = coefficients[1];
  const double c = coefficients[2];
  if (std::fabs(a) <= std::numeric_limits<double>::epsilon()) {
    return -1;
  }

  double delta = b * b - 4.0 * a * c;
  if (delta < 0.0) {
    return -1;
  }

  roots->first = (0.0 - b + std::sqrt(delta)) / (2.0 * a);
  roots->second = (0.0 - b - std::sqrt(delta)) / (2.0 * a);
  return 0;
}

void TranslatePoint(const double translate_x, const double translate_y,
                    TrajectoryPoint* point) {
  if (point == nullptr || !point->has_path_point()) {
    AERROR << "Point is nullptr or has NO path_point.";
  }
  const double original_x = point->path_point().x();
  const double original_y = point->path_point().y();
  point->mutable_path_point()->set_x(original_x + translate_x);
  point->mutable_path_point()->set_y(original_y + translate_y);
}

}  // namespace util
}  // namespace prediction
}  // namespace apollo
