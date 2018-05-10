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
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

DiscretizedPath::DiscretizedPath(
    const std::vector<common::PathPoint> &path_points) {
  path_points_ = path_points;
}

void DiscretizedPath::set_path_points(
    const std::vector<common::PathPoint> &path_points) {
  path_points_ = path_points;
}

double DiscretizedPath::Length() const {
  if (path_points_.empty()) {
    return 0.0;
  }
  return path_points_.back().s() - path_points_.front().s();
}

common::PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  CHECK(!path_points_.empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == path_points_.begin()) {
    return path_points_.front();
  }
  if (it_lower == path_points_.end()) {
    return path_points_.back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                           *it_lower, path_s);
}

const std::vector<common::PathPoint> &DiscretizedPath::path_points() const {
  return path_points_;
}

std::uint32_t DiscretizedPath::NumOfPoints() const {
  return path_points_.size();
}

const common::PathPoint &DiscretizedPath::StartPoint() const {
  CHECK(!path_points_.empty());
  return path_points_.front();
}

const common::PathPoint &DiscretizedPath::EndPoint() const {
  CHECK(!path_points_.empty());
  return path_points_.back();
}

void DiscretizedPath::Clear() { path_points_.clear(); }

std::vector<common::PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const common::PathPoint &tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(path_points_.begin(), path_points_.end(), path_s,
                          func);
}

}  // namespace planning
}  // namespace apollo
