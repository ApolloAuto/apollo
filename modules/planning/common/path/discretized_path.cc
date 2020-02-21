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
 * @file path.cc
 **/

#include "modules/planning/common/path/discretized_path.h"

#include <algorithm>
#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;

DiscretizedPath::DiscretizedPath(std::vector<PathPoint> path_points)
    : std::vector<PathPoint>(std::move(path_points)) {}

double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  ACHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                           *it_lower, path_s);
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const PathPoint &tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

PathPoint DiscretizedPath::EvaluateReverse(const double path_s) const {
  ACHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_upper - 1),
                                                           *it_upper, path_s);
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
    const double path_s) const {
  auto func = [](const double path_s, const PathPoint &tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}

}  // namespace planning
}  // namespace apollo
