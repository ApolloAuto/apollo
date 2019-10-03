/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 **/

#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::math::LineSegment2d;

STObstaclesProcessor::STObstaclesProcessor(
    const double planning_distance, const double planning_time,
    const PathData& path_data)
    : planning_time_(planning_time),
      planning_distance_(planning_distance),
      path_data_(path_data) {}

Status STObstaclesProcessor::MapObstaclesToSTBoundaries(
    PathDecision* const path_decision) {
  // Sanity checks.
  CHECK_NOTNULL(path_decision);
  CHECK_GT(planning_time_, 0.0);
  CHECK_GT(planning_distance_, 0.0);
  CHECK_GT(path_data_.discretized_path().size(), 1);

  // Go through every obstacle.
  return Status::OK();
}

// TODO(jiacheng): implement this.
std::pair<double, double> STObstaclesProcessor::GetRegularBoundaryFromObstacles(
    double t) {
  return {0.0, 0.0};
}

// TODO(jiacheng): implement this.
std::pair<double, double>
STObstaclesProcessor::GetFallbackBoundaryFromObstacles(double t) {
  return {0.0, 0.0};
}

///////////////////////////////////////////////////////////////////////////////
// Private helper functions.

bool STObstaclesProcessor::ComputeObstacleSTBoundary(const Obstacle& obstacle) {
  // std::vector<STPoint> lower_points;
  // std::vector<STPoint> upper_points;
  // const auto& adc_path_points = path_data_.discretized_path();
  // const auto& obs_trajectory = obstacle.Trajectory();

  // if (obs_trajectory.trajectory_point_size() == 0) {
  //   // Processing a static obstacle.
  //   // Sanity checks.
  //   if (!obstacle.IsStatic()) {
  //     AWARN << "Non-static obstacle[" << obstacle.Id()
  //           << "] has NO prediction trajectory."
  //           << obstacle.Perception().ShortDebugString();
  //   }
  // } else {
  //   // Processing a dynamic obstacle.
  // }
  return true;
}

bool STObstaclesProcessor::GetOverlappingS(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double adc_l_buffer,
    std::pair<double, double>* const overlapping_s) {
  return true;
}

int STObstaclesProcessor::GetSBoundingPathPointIndex(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double s_thresh,
    const bool is_lower_bound, const int start_idx, const int end_idx) {
  if (start_idx == end_idx) {
    if (IsPathPointAwayFromObstacle(adc_path_points[start_idx],
            adc_path_points[start_idx+1], obstacle_instance, s_thresh,
            is_lower_bound))
      return start_idx;
    else
      return -1;
  }

  if (is_lower_bound) {
    int mid_idx = (start_idx + end_idx - 1) / 2 + 1;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
            adc_path_points[mid_idx+1], obstacle_instance, s_thresh,
            is_lower_bound))
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_lower_bound,
                                        mid_idx, end_idx);
    else
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_lower_bound,
                                        start_idx, mid_idx-1);
  } else {
    int mid_idx = (start_idx + end_idx) / 2;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
            adc_path_points[mid_idx+1], obstacle_instance, s_thresh,
            is_lower_bound))
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_lower_bound,
                                        start_idx, mid_idx);
    else
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_lower_bound,
                                        mid_idx+1, end_idx);
  }
}

bool STObstaclesProcessor::IsPathPointAwayFromObstacle(
    const PathPoint& path_point, const PathPoint& direction_point,
    const Box2d& obs_box, const double s_thresh, const bool is_before) {
  Vec2d path_pt(path_point.x(), path_point.y());
  Vec2d dir_pt(direction_point.x(), direction_point.y());
  LineSegment2d path_dir_lineseg(path_pt, dir_pt);
  LineSegment2d normal_line_seg(
      path_pt, path_dir_lineseg.rotate(M_PI_2));

  auto corner_points = obs_box.GetAllCorners();
  for (const auto& corner_pt : corner_points) {
    Vec2d normal_line_ft_pt;
    normal_line_seg.GetPerpendicularFoot(corner_pt, &normal_line_ft_pt);
    Vec2d path_dir_unit_vec = path_dir_lineseg.unit_direction();
    Vec2d perpendicular_vec = corner_pt - normal_line_ft_pt;
    double corner_pt_s_dist = path_dir_unit_vec.InnerProd(perpendicular_vec);
    if (is_before && corner_pt_s_dist < s_thresh)
      return false;
    if (!is_before && corner_pt_s_dist > -s_thresh)
      return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
