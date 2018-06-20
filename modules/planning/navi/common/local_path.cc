/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file
 * @brief This file provides the implementation of the class "LocalPath".
 */
#include "modules/planning/navi/common/local_path.h"

#include <memory>
#include <vector>

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

LocalPath::LocalPath(const std::vector<common::PathPoint> &path_points)
    : path_points_(path_points) {
  for (size_t i = 0; i < path_points_.size(); ++i) {
    points_.emplace_back(path_points_[i].x(), path_points_[i].y());
  }
}

bool LocalPath::GetInitY(double *y_val) {
  // DCHECK_NOTNULL(y_val);
  if (points_.empty()) {
    return false;
  }
  *y_val = points_[0].y();
  return true;
}

void LocalPath::Shift(const double dist) {
  points_.clear();
  for (size_t i = 0; i < path_points_.size(); ++i) {
    path_points_[i].set_y(path_points_[i].y() + dist);
    points_.emplace_back(path_points_[i].x(), path_points_[i].y());
  }
}

void LocalPath::Cut(const double dist) {
  return;
}

void LocalPath::Resample() {
  return;
}

void LocalPath::Merge(const common::Path &local_path, const double weight) {
  points_.clear();
  for (size_t i = 0; i < path_points_.size(); ++i) {
    auto y = path_points_[i].y();
    if (i < static_cast<size_t>(local_path.path_point_size())) {
      auto y2 = local_path.path_point(i).y() * weight;
      path_points_[i].set_y((y + y2) / (1 + weight));
    }
    points_.emplace_back(path_points_[i].x(), path_points_[i].y());
  }
}

}  // namespace planning
}  // namespace apollo
