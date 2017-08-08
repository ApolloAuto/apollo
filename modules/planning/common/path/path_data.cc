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
 * @file path_data.cc
 **/

#include "modules/planning/common/path/path_data.h"

#include <algorithm>
#include <limits>

#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using SLPoint = apollo::common::SLPoint;
using Vec2d = apollo::common::math::Vec2d;

void PathData::set_discretized_path(const DiscretizedPath &path) {
  discretized_path_ = path;
}

void PathData::set_frenet_path(const FrenetFramePath &frenet_path) {
  frenet_path_ = frenet_path;
}

void PathData::set_discretized_path(
    const std::vector<common::PathPoint> &path_points) {
  discretized_path_.set_points(path_points);
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}

void PathData::set_reference_line(const ReferenceLine *reference_line) {
  reference_line_ = reference_line;
}

bool PathData::get_path_point_with_path_s(
    const double s, common::PathPoint *const path_point) const {
  *path_point = discretized_path_.evaluate_linear_approximation(s);
  return true;
}

bool PathData::get_path_point_with_ref_s(
    const double ref_s, common::PathPoint *const path_point) const {
  DCHECK_NOTNULL(reference_line_);
  DCHECK_NOTNULL(path_point);
  *path_point = discretized_path_.start_point();
  double shortest_distance = std::numeric_limits<double>::max();
  const double kDistanceEpsilon = 1e-3;
  for (const auto &curr_path_point : discretized_path_.points()) {
    SLPoint sl;
    if (!reference_line_->get_point_in_frenet_frame(
            Vec2d(curr_path_point.x(), curr_path_point.y()), &sl)) {
      AERROR << "Fail to get point in frenet from.";
      return false;
    }
    const double curr_distance = std::fabs(sl.s() - ref_s);

    if (curr_distance < kDistanceEpsilon) {
      path_point->CopyFrom(curr_path_point);
      return true;
    }
    if (curr_distance < shortest_distance) {
      path_point->CopyFrom(curr_path_point);
      shortest_distance = curr_distance;
    }
  }
  return true;
}

void PathData::Clear() {
  discretized_path_ = DiscretizedPath();
  frenet_path_ = FrenetFramePath();
  reference_line_ = nullptr;
}

std::string PathData::DebugString() const {
  const auto &path_points = discretized_path_.points();
  const auto limit =
      std::min(path_points.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));

  return apollo::common::util::StrCat(
      "[\n", apollo::common::util::PrintDebugStringIter(
                 path_points.begin(), path_points.begin() + limit, ",\n"),
      "]\n");
}

}  // namespace planning
}  // namespace apollo
