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
 * @brief This file provides the implementation of the class
 * "TrajectorySmoother".
 */
#include "modules/tools/navi_generator/backend/util/trajectory_smoother.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line_smoother.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"

namespace apollo {
namespace navi_generator {
namespace util {

using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;
using apollo::hdmap::MapPathPoint;
using apollo::planning::AnchorPoint;
using apollo::planning::QpSplineReferenceLineSmoother;
using apollo::planning::ReferenceLine;
using apollo::planning::ReferencePoint;

TrajectorySmoother::TrajectorySmoother() {
  TrajectoryUtilConfig util_config;
  if (!common::util::GetProtoFromFile(FLAGS_trajectory_util_config_filename,
                                      &util_config)) {
    AERROR << "Failed to read the trajectory smoother config file: "
           << FLAGS_trajectory_util_config_filename;
  }

  traj_smoother_config_ = util_config.smoother_config();

  if (!common::util::GetProtoFromFile(
          traj_smoother_config_.smoother_config_filename(),
          &smoother_config_)) {
    AERROR << "Failed to read the smoothing algorithm's config file: "
           << traj_smoother_config_.smoother_config_filename();
  }
}

bool TrajectorySmoother::Import(const std::string& filename) {
  filename_ = filename;
  std::ifstream ifs(filename.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    AERROR << "Can't open the raw trajecotry's file: " << filename;
    return false;
  }

  std::string point_str;
  while (std::getline(ifs, point_str)) {
    std::size_t idx = point_str.find(',');
    if (idx == std::string::npos) {
      continue;
    }
    auto x_str = point_str.substr(0, idx);
    auto y_str = point_str.substr(idx + 1);
    raw_points_.emplace_back(std::stod(x_str), std::stod(y_str));
  }

  return true;
}

bool TrajectorySmoother::Smooth() {
  if (raw_points_.size() <= 2) {
    AERROR << "the original point size is " << raw_points_.size();
    return false;
  }
  std::size_t i = 1;
  {
    std::vector<ReferencePoint> ref_points;
    double s = 0.0;
    for (; s < traj_smoother_config_.smooth_length() && i < raw_points_.size();
         ++i) {
      LineSegment2d segment(raw_points_[i - 1], raw_points_[i]);
      ref_points.emplace_back(MapPathPoint(raw_points_[i], segment.heading()),
                              0.0, 0.0);
      s += segment.length();
    }
    ReferenceLine init_ref(ref_points);
    auto smoother_ptr =
        std::make_unique<QpSplineReferenceLineSmoother>(smoother_config_);
    std::vector<AnchorPoint> anchors;
    if (!CreateAnchorPoints(init_ref.reference_points().front(), init_ref,
                            &anchors)) {
      AERROR << "Can't create anchor points.";
      return false;
    }

    smoother_ptr->SetAnchorPoints(anchors);
    ReferenceLine smoothed_init_ref;
    if (!smoother_ptr->Smooth(init_ref, &smoothed_init_ref)) {
      AERROR << "smooth initial reference line failed";
      return false;
    }
    ref_points_ = smoothed_init_ref.reference_points();
  }
  for (; i < raw_points_.size(); ++i) {
    double s = 0.0;
    std::size_t j = ref_points_.size() - 1;
    while (j > 0 && s < traj_smoother_config_.smooth_length() / 2.0) {
      s += DistanceXY(ref_points_[j - 1], ref_points_[j]);
      --j;
    }
    ReferenceLine prev_half_ref(ref_points_.begin() + j, ref_points_.end());
    ref_points_.erase(ref_points_.begin() + j, ref_points_.end());
    common::SLPoint sl;
    prev_half_ref.XYToSL(raw_points_[i], &sl);
    while (sl.s() <= prev_half_ref.Length() && i + 1 < raw_points_.size()) {
      prev_half_ref.XYToSL(raw_points_[i + 1], &sl);
      ++i;
    }
    s = 0.0;
    j = i;
    auto ref_points = prev_half_ref.reference_points();
    while (j + 1 < raw_points_.size() &&
           s < traj_smoother_config_.smooth_length() / 2.0) {
      Vec2d vec = raw_points_[j + 1] - raw_points_[j];
      s += vec.Length();
      ref_points.emplace_back(MapPathPoint(raw_points_[j], vec.Angle()), 0.0,
                              0.0);
      ++j;
    }
    i = j;
    ReferenceLine local_ref(ref_points);
    std::vector<AnchorPoint> anchors;
    if (!CreateAnchorPoints(ref_points.front(), local_ref, &anchors)) {
      AERROR << "Can't create anchor points.";
      return false;
    }

    auto smoother_ptr =
        std::make_unique<QpSplineReferenceLineSmoother>(smoother_config_);
    smoother_ptr->SetAnchorPoints(anchors);
    ReferenceLine smoothed_local_ref;
    if (!smoother_ptr->Smooth(local_ref, &smoothed_local_ref)) {
      AERROR << "Failed to smooth reference line";
      return false;
    }
    ref_points_.insert(ref_points_.end(),
                       smoothed_local_ref.reference_points().begin(),
                       smoothed_local_ref.reference_points().end());
  }
  return true;
}

bool TrajectorySmoother::Export(const std::string& filename) {
  if (ref_points_.empty()) {
    AERROR << "There aren't any smoothed points to output.";
    return false;
  }

  std::ofstream ofs(filename.c_str());
  if (!ofs.is_open()) {
    AERROR << "Failed to open the output file: " << filename;
    return false;
  }
  ofs.precision(6);
  double s = 0.0;
  // skip the first point and the last point
  for (std::size_t i = 1; i + 1 < ref_points_.size(); ++i) {
    const auto& point = ref_points_[i];
    ofs << std::fixed << "{\"kappa\": " << point.kappa() << ", \"s\": " << s
        << ", \"theta\": " << point.heading() << ", \"x\":" << point.x()
        << ", \"y\":" << point.y() << ", \"dkappa\":" << point.dkappa() << "}"
        << std::endl;
    s += DistanceXY(point, ref_points_[i + 1]);
  }
  ofs.close();
  AINFO << "The smoothed result is saved to the file: " << filename;

  return true;
}

bool TrajectorySmoother::CreateAnchorPoints(
    const planning::ReferencePoint& init_point,
    const planning::ReferenceLine& ref_line,
    std::vector<planning::AnchorPoint>* anchor_points) {
  CHECK_NOTNULL(anchor_points);

  int num_of_anchors = std::max(
      2, static_cast<int>(ref_line.Length() /
                              smoother_config_.max_constraint_interval() +
                          0.5));
  std::vector<double> anchor_s;
  common::util::uniform_slice(0.0, ref_line.Length(), num_of_anchors - 1,
                              &anchor_s);
  common::SLPoint sl;
  if (!ref_line.XYToSL(Vec2d(init_point.x(), init_point.y()), &sl)) {
    AERROR << "Failed to project init point to reference line";
    return false;
  }
  bool set_init_point = false;
  for (const double s : anchor_s) {
    if (s + smoother_config_.max_constraint_interval() / 2.0 < sl.s()) {
      continue;
    }
    ReferencePoint ref_point;
    if (!set_init_point) {
      set_init_point = true;
      ref_point = init_point;
    } else {
      ref_point = ref_line.GetReferencePoint(s);
    }
    AnchorPoint anchor;
    anchor.path_point.set_x(ref_point.x());
    anchor.path_point.set_y(ref_point.y());
    anchor.path_point.set_z(0.0);
    anchor.path_point.set_s(s);
    anchor.path_point.set_theta(ref_point.heading());
    anchor.path_point.set_kappa(ref_point.kappa());
    anchor.lateral_bound = smoother_config_.lateral_boundary_bound();
    anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
    anchor_points->emplace_back(anchor);
  }
  anchor_points->front().longitudinal_bound = 0;
  anchor_points->front().lateral_bound = 0;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 0;
  anchor_points->back().lateral_bound = 0;
  anchor_points->back().enforced = true;
  return true;
}
}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
