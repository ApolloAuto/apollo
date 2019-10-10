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
#include <tuple>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

STObstaclesProcessor::STObstaclesProcessor(const double planning_distance,
                                           const double planning_time,
                                           const PathData& path_data)
    : planning_time_(planning_time),
      planning_distance_(planning_distance),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  adc_path_init_s_ = path_data_.discretized_path().front().s();
}

// TODO(jiacheng):
//  1. Properly deal with clear-zone type obstacles.
//  2. Further speed up by early terminating processing non-related obstacles.
Status STObstaclesProcessor::MapObstaclesToSTBoundaries(
    PathDecision* const path_decision) {
  // Sanity checks.
  CHECK_NOTNULL(path_decision);
  CHECK_GT(planning_time_, 0.0);
  CHECK_GT(planning_distance_, 0.0);
  CHECK_GT(path_data_.discretized_path().size(), 1);
  obs_id_to_st_boundary_.clear();

  // Go through every obstacle.
  std::tuple<std::string, STBoundary, Obstacle*> closest_stop_obstacle;
  std::get<0>(closest_stop_obstacle) = "NULL";
  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    CHECK_NOTNULL(obs_ptr);

    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    if (!ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points)) {
      // Obstacle doesn't appear on ST-Graph.
      continue;
    }
    CHECK_GT(lower_points.size(), 1);
    CHECK_GT(upper_points.size(), 1);
    auto boundary = STBoundary::CreateInstanceAccurate(
        lower_points, upper_points);
    boundary.set_id(obs_ptr->Id());
    if (obs_ptr->Trajectory().trajectory_point_size() == 0) {
      // Obstacle is static.
      if (std::get<0>(closest_stop_obstacle) == "NULL" ||
          std::get<1>(closest_stop_obstacle).bottom_left_point().s() >
              boundary.bottom_left_point().s()) {
        // If this static obstacle is closer for ADC to stop, record it.
        closest_stop_obstacle = std::make_tuple(
            obs_ptr->Id(), boundary, obs_ptr);
      }
    } else {
      // Obstacle is dynamic.
      if (boundary.bottom_left_point().s() - adc_path_init_s_
              < kSIgnoreThreshold) {
        // Ignore backward obstacles.
        // TODO(jiacheng): don't ignore if ADC is in dangerous segments.
        continue;
      }
      obs_id_to_st_boundary_[obs_ptr->Id()] = boundary;
      obs_ptr->set_path_st_boundary(boundary);
    }
  }
  // Only retain the closest static obstacle.
  if (std::get<0>(closest_stop_obstacle) != "NULL") {
    obs_id_to_st_boundary_[std::get<0>(closest_stop_obstacle)] =
        std::get<1>(closest_stop_obstacle);
    std::get<2>(closest_stop_obstacle)->set_path_st_boundary(
        std::get<1>(closest_stop_obstacle));
  }

  return Status::OK();
}

std::unordered_map<std::string, STBoundary>
STObstaclesProcessor::GetAllSTBoundaries() {
  return obs_id_to_st_boundary_;
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

bool STObstaclesProcessor::ComputeObstacleSTBoundary(
    const Obstacle& obstacle,
    std::vector<STPoint>* const lower_points,
    std::vector<STPoint>* const upper_points) {
  lower_points->clear();
  upper_points->clear();
  const auto& adc_path_points = path_data_.discretized_path();
  const auto& obs_trajectory = obstacle.Trajectory();

  if (obs_trajectory.trajectory_point_size() == 0) {
    // Processing a static obstacle.
    // Sanity checks.
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    // Get the overlapping s between ADC path and obstacle's perception box.
    const Box2d& obs_box = obstacle.PerceptionBoundingBox();
    std::pair<double, double> overlapping_s;
    if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
        &overlapping_s)) {
      lower_points->emplace_back(overlapping_s.first, 0.0);
      lower_points->emplace_back(overlapping_s.first, planning_time_);
      upper_points->emplace_back(overlapping_s.second, 0.0);
      upper_points->emplace_back(overlapping_s.second, planning_time_);
    }
  } else {
    // Processing a dynamic obstacle.
    // Go through every occurence of the obstacle at all timesteps, and
    // figure out the overlapping s-max and s-min one by one.
    for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {
      // TODO(jiacheng): Currently, if the obstacle overlaps with ADC at
      // disjoint segments (happens very rarely), we merge them into one.
      // In the future, this could be considered in greater details rather
      // than being approximated.
      const Box2d& obs_box = obstacle.GetBoundingBox(obs_traj_pt);
      std::pair<double, double> overlapping_s;
      if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
          &overlapping_s)) {
        lower_points->emplace_back(
            overlapping_s.first, obs_traj_pt.relative_time());
        upper_points->emplace_back(
            overlapping_s.second, obs_traj_pt.relative_time());
      }
    }
    if (lower_points->size() == 1) {
      lower_points->emplace_back(
          lower_points->front().s(), lower_points->front().t() + 0.1);
      upper_points->emplace_back(
          upper_points->front().s(), upper_points->front().t() + 0.1);
    }
  }

  return (!lower_points->empty() && !upper_points->empty());
}

bool STObstaclesProcessor::GetOverlappingS(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double adc_l_buffer,
    std::pair<double, double>* const overlapping_s) {
  // Locate the possible range to search in details.
  int pt_before_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.front_edge_to_center(),
      true, 0, static_cast<int>(adc_path_points.size()) - 2);
  int pt_after_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.back_edge_to_center(),
      false, 0, static_cast<int>(adc_path_points.size()) - 2);
  if (pt_before_idx == static_cast<int>(adc_path_points.size()) - 2) {
    return false;
  }
  if (pt_after_idx == 0) {
    return false;
  }

  if (pt_before_idx == -1) {
    pt_before_idx = 0;
  }
  if (pt_after_idx == -1) {
    pt_after_idx = static_cast<int>(adc_path_points.size()) - 2;
  }
  CHECK(pt_before_idx < pt_after_idx);

  // Detailed searching.
  bool has_overlapping = false;
  for (int i = pt_before_idx; i <= pt_after_idx; ++i) {
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->first = adc_path_points[std::max(i - 1, 0)].s();
      has_overlapping = true;
      break;
    }
  }
  if (!has_overlapping) return false;

  for (int i = pt_after_idx; i >= pt_before_idx; --i) {
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->second = adc_path_points[i + 1].s();
      break;
    }
  }
  return true;
}

int STObstaclesProcessor::GetSBoundingPathPointIndex(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double s_thresh, const bool is_before,
    const int start_idx, const int end_idx) {
  if (start_idx == end_idx) {
    if (IsPathPointAwayFromObstacle(adc_path_points[start_idx],
                                    adc_path_points[start_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return start_idx;
    } else {
      return -1;
    }
  }

  if (is_before) {
    int mid_idx = (start_idx + end_idx - 1) / 2 + 1;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx, end_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx - 1);
    }
  } else {
    int mid_idx = (start_idx + end_idx) / 2;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx + 1,
                                        end_idx);
    }
  }
}

bool STObstaclesProcessor::IsPathPointAwayFromObstacle(
    const PathPoint& path_point, const PathPoint& direction_point,
    const Box2d& obs_box, const double s_thresh, const bool is_before) {
  Vec2d path_pt(path_point.x(), path_point.y());
  Vec2d dir_pt(direction_point.x(), direction_point.y());
  LineSegment2d path_dir_lineseg(path_pt, dir_pt);
  LineSegment2d normal_line_seg(path_pt, path_dir_lineseg.rotate(M_PI_2));

  auto corner_points = obs_box.GetAllCorners();
  for (const auto& corner_pt : corner_points) {
    Vec2d normal_line_ft_pt;
    normal_line_seg.GetPerpendicularFoot(corner_pt, &normal_line_ft_pt);
    Vec2d path_dir_unit_vec = path_dir_lineseg.unit_direction();
    Vec2d perpendicular_vec = corner_pt - normal_line_ft_pt;
    double corner_pt_s_dist = path_dir_unit_vec.InnerProd(perpendicular_vec);
    if (is_before && corner_pt_s_dist < s_thresh) return false;
    if (!is_before && corner_pt_s_dist > -s_thresh) return false;
  }
  return true;
}

bool STObstaclesProcessor::IsADCOverlappingWithObstacle(
    const PathPoint& adc_path_point, const Box2d& obs_box,
    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(adc_path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

}  // namespace planning
}  // namespace apollo
