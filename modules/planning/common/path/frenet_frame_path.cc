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
 * @file frenet_frame_path.cc
 **/
#include "modules/planning/common/path/frenet_frame_path.h"

#include <utility>

#include "modules/common/log.h"
#include "modules/common/proto/path_point.pb.h"

namespace apollo {
namespace planning {

FrenetFramePath::FrenetFramePath(
    std::vector<common::FrenetFramePoint> sl_points) {
  points_ = std::move(sl_points);
}

void FrenetFramePath::set_frenet_points(
    const std::vector<common::FrenetFramePoint> &points) {
  points_ = points;
}

std::vector<common::FrenetFramePoint> *FrenetFramePath::mutable_points() {
  return &points_;
}

const std::vector<common::FrenetFramePoint> &FrenetFramePath::points() const {
  return points_;
}

std::uint32_t FrenetFramePath::number_of_points() const {
  return points_.size();
}

const common::FrenetFramePoint &FrenetFramePath::point_at(
    const std::uint32_t index) const {
  CHECK_LT(index, points_.size());
  return points_[index];
}

common::FrenetFramePoint &FrenetFramePath::point_at(const std::uint32_t index) {
  CHECK_LT(index, points_.size());
  return points_[index];
}

}  // namespace planning
}  // namespace apollo
