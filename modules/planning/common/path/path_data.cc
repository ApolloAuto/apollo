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

#include <sstream>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

void PathData::set_path(const DiscretizedPath &path) { path_ = path; }

void PathData::set_frenet_path(const FrenetFramePath &frenet_path) {
  frenet_path_ = frenet_path;
}

DiscretizedPath *PathData::mutable_path() { return &path_; }

const DiscretizedPath &PathData::path() const { return path_; }

FrenetFramePath *PathData::mutable_frenet_frame_path() { return &frenet_path_; }

const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}

bool PathData::get_path_point_with_path_s(
    const double s, common::PathPoint *const path_point) const {
  const auto &path_points = path_.path_points();

  if (path_points.size() < 2 || s < path_points.front().s() ||
      path_points.back().s() < s) {
    return false;
  }

  auto comp = [](const common::PathPoint &path_point, const double s) {
    return path_point.s() < s;
  };

  auto it_lower =
      std::lower_bound(path_points.begin(), path_points.end(), s, comp);
  if (it_lower == path_points.begin()) {
    *path_point = *it_lower;
  } else {
    double s0 = (it_lower - 1)->s();
    double s1 = it_lower->s();
    CHECK_LT(s0, s1);
    *path_point =
        util::interpolate_linear_approximation(*(it_lower - 1), *it_lower, s);
  }
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
    *path_point = path_.path_points().front();
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

std::string PathData::DebugString() const {
  std::ostringstream sout;
  sout << "[" << std::endl;
  const auto &path_points = path_.path_points();
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
