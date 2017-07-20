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

#include <utility>

#include "modules/common/log.h"
#include "modules/planning/common/path/path_point_util.h"

namespace apollo {
namespace planning {

DiscretizedPath::DiscretizedPath(std::vector<common::PathPoint> path_points) {
  path_points_ = std::move(path_points);
}

common::PathPoint DiscretizedPath::evaluate(const double param) const {
  CHECK_GT(path_points_.size(), 1);
  CHECK(path_points_.front().s() <= param && path_points_.back().s() <= param);

  auto it_lower = query_lower_bound(param);
  if (it_lower == path_points_.begin()) {
    return path_points_.front();
  }

  return util::interpolate(*(it_lower - 1), *it_lower, param);
}

double DiscretizedPath::param_length() const {
  if (path_points_.empty()) {
    return 0.0;
  }
  return path_points_.back().s() - path_points_.front().s();
}

int DiscretizedPath::query_closest_point(const double param) const {
  if (path_points_.empty()) {
    return -1;
  }
  auto it_lower = query_lower_bound(param);

  if (it_lower == path_points_.begin()) {
    return 0;
  }
  if (it_lower == path_points_.end()) {
    return path_points_.size() - 1;
  }

  double d0 = param - (it_lower - 1)->s();
  double d1 = it_lower->s() - param;

  if (d0 < d1) {
    return static_cast<int>(it_lower - path_points_.begin()) - 1;
  } else {
    return static_cast<int>(it_lower - path_points_.begin());
  }
}

std::vector<common::PathPoint>* DiscretizedPath::mutable_path_points() {
  return &path_points_;
}

const std::vector<common::PathPoint>& DiscretizedPath::path_points() const {
  return path_points_;
}

std::size_t DiscretizedPath::num_of_points() const {
  return path_points_.size();
}

const common::PathPoint& DiscretizedPath::path_point_at(
    const std::size_t index) const {
  CHECK_LT(index, path_points_.size());
  return path_points_[index];
}

common::PathPoint DiscretizedPath::start_point() const {
  CHECK(!path_points_.empty());
  return path_points_.front();
}

common::PathPoint DiscretizedPath::end_point() const {
  CHECK(!path_points_.empty());
  return path_points_.back();
}

common::PathPoint& DiscretizedPath::path_point_at(const std::size_t index) {
  CHECK_LT(index, path_points_.size());
  return path_points_[index];
}

std::vector<common::PathPoint>::const_iterator
DiscretizedPath::query_lower_bound(const double param) const {
  auto func = [](const common::PathPoint& tp, const double param) {
    return tp.s() < param;
  };
  return std::lower_bound(path_points_.begin(), path_points_.end(), param,
                          func);
}

}  // namespace planning
}  // namespace apollo
