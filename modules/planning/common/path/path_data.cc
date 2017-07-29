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
#include <sstream>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

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

bool PathData::get_path_point_with_path_s(
    const double s, common::PathPoint *const path_point) const {
  *path_point = discretized_path_.evaluate_linear_approximation(s);
  return true;
}

bool PathData::get_path_point_with_ref_s(
    const double ref_s, common::PathPoint *const path_point) const {
  const auto &frenet_points = frenet_path_.points();
  if (frenet_points.size() < 2 || ref_s < frenet_points.front().s() ||
      frenet_points.back().s() < ref_s) {
    return false;
  }

  auto comp = [](const common::FrenetFramePoint &frenet_point,
                 const double ref_s) { return frenet_point.s() < ref_s; };

  auto it_lower =
      std::lower_bound(frenet_points.begin(), frenet_points.end(), ref_s, comp);
  if (it_lower == frenet_points.begin()) {
    *path_point = discretized_path_.points().front();
  } else {
    //        std::uint32_t index_lower = (std::uint32_t)(it_lower -
    //        frenet_points.begin());
    //
    //        double ref_s0 = (it_lower - 1)->s();
    //        double ref_s1 = it_lower->s();
    //
    //        CHECK_LT(ref_s0, ref_s1);
    //        double weight = (ref_s - ref_s0) / (ref_s1 - ref_s0);
    //        *path_point =
    //        common::PathPoint::interpolate_linear_approximation(path_.path_point_at(index_lower
    //        - 1),
    //                path_.path_point_at(index_lower), weight);
  }
  return true;
}

void PathData::Clear() {
  discretized_path_ = DiscretizedPath();
  frenet_path_ = FrenetFramePath();
}

std::string PathData::DebugString() const {
  std::ostringstream sout;
  sout << "[" << std::endl;
  const auto &path_points = discretized_path_.points();
  for (std::size_t i = 0;
       i < path_points.size() &&
       i < static_cast<std::size_t>(FLAGS_trajectory_point_num_for_debug);
       ++i) {
    if (i > 0) {
      sout << "," << std::endl;
    }
    sout << path_points[i].DebugString();
  }
  sout << "]" << std::endl;
  sout.flush();
  return sout.str();
}

}  // namespace planning
}  // namespace apollo
