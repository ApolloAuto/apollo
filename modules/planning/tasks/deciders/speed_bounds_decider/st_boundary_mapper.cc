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
 * @file
 **/

#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

STBoundaryMapper::STBoundaryMapper(
    const SpeedBoundsDeciderConfig& config, const ReferenceLine& reference_line,
    const PathData& path_data, const double planning_distance,
    const double planning_time,
    const std::shared_ptr<DependencyInjector>& injector)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_max_distance_(planning_distance),
      planning_max_time_(planning_time),
      injector_(injector) {}

Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision) const {
  // Sanity checks.
  CHECK_GT(planning_max_time_, 0.0);
  if (path_data_.discretized_path().size() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().size() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  // Go through every obstacle.
  Obstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();
  for (const auto* ptr_obstacle_item : path_decision->obstacles().Items()) {
    Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());
    ACHECK(ptr_obstacle != nullptr);

    // If no longitudinal decision has been made, then plot it onto ST-graph.
    if (!ptr_obstacle->HasLongitudinalDecision()) {
      ComputeSTBoundary(ptr_obstacle);
      continue;
    }

    // If there is a longitudinal decision, then fine-tune boundary.
    const auto& decision = ptr_obstacle->LongitudinalDecision();
    if (decision.has_stop()) {
      // 1. Store the closest stop fence info.
      // TODO(all): store ref. s value in stop decision; refine the code then.
      common::SLPoint stop_sl_point;
      reference_line_.XYToSL(decision.stop().stop_point(), &stop_sl_point);
      const double stop_s = stop_sl_point.s();

      if (stop_s < min_stop_s) {
        stop_obstacle = ptr_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      // 2. Depending on the longitudinal overtake/yield decision,
      //    fine-tune the upper/lower st-boundary of related obstacles.
      ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
    } else if (!decision.has_ignore()) {
      // 3. Ignore those unrelated obstacles.
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }
  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      const std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  return Status::OK();
}

bool STBoundaryMapper::MapStopDecision(
    Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const {
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";
  common::SLPoint stop_sl_point;
  reference_line_.XYToSL(stop_decision.stop().stop_point(), &stop_sl_point);

  double st_stop_s = 0.0;
  const double stop_ref_s =
      stop_sl_point.s() - vehicle_param_.front_edge_to_center();

  if (stop_ref_s > path_data_.frenet_frame_path().back().s()) {
    st_stop_s = path_data_.discretized_path().back().s() +
                (stop_ref_s - path_data_.frenet_frame_path().back().s());
  } else {
    PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      return false;
    }
    st_stop_s = stop_point.s();
  }

  const double s_min = std::fmax(0.0, st_stop_s);
  const double s_max = std::fmax(
      s_min, std::fmax(planning_max_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_max_time_),
      STPoint(s_max + speed_bounds_config_.boundary_buffer(),
              planning_max_time_));
  auto boundary = STBoundary(point_pairs);
  boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(speed_bounds_config_.boundary_buffer());
  boundary.set_id(stop_obstacle->Id());
  stop_obstacle->set_path_st_boundary(boundary);
  return true;
}

void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) const {
  if (FLAGS_use_st_drivable_boundary) {
    return;
  }
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                &upper_points, &lower_points)) {
    return;
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
  boundary.set_id(obstacle->Id());

  // TODO(all): potential bug here.
  const auto& prev_st_boundary = obstacle->path_st_boundary();
  const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }

  obstacle->set_path_st_boundary(boundary);
}

bool STBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  // Sanity checks.
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }

  const auto* planning_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_change_lane();

  double l_buffer =
      planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE
          ? FLAGS_lane_change_obstacle_nudge_l_buffer
          : FLAGS_nonstatic_obstacle_nudge_l_buffer;

  // Draw the given obstacle on the ST-graph.
  const auto& trajectory = obstacle.Trajectory();
  if (trajectory.trajectory_point().empty()) {
    // For those with no predicted trajectories, just map the obstacle's
    // current position to ST-graph and always assume it's static.
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    for (const auto& curr_point_on_path : path_points) {
      if (curr_point_on_path.s() > planning_max_distance_) {
        break;
      }

      const Box2d& obs_box = obstacle.PerceptionBoundingBox();
      if (CheckOverlap(curr_point_on_path, obs_box, l_buffer)) {
        // If there is overlapping, then plot it on ST-graph.
        const double backward_distance = -vehicle_param_.front_edge_to_center();
        const double forward_distance = obs_box.length();
        double low_s =
            std::fmax(0.0, curr_point_on_path.s() + backward_distance);
        double high_s = std::fmin(planning_max_distance_,
                                  curr_point_on_path.s() + forward_distance);
        // It is an unrotated rectangle appearing on the ST-graph.
        // TODO(jiacheng): reconsider the backward_distance, it might be
        // unnecessary, but forward_distance is indeed meaningful though.
        lower_points->emplace_back(low_s, 0.0);
        lower_points->emplace_back(low_s, planning_max_time_);
        upper_points->emplace_back(high_s, 0.0);
        upper_points->emplace_back(high_s, planning_max_time_);
        break;
      }
    }
  } else {
    // For those with predicted trajectories (moving obstacles):
    // 1. Subsample to reduce computation time.
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
    if (path_points.size() > 2 * default_num_point) {
      const auto ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path = DiscretizedPath(std::move(sampled_path_points));
    } else {
      discretized_path = DiscretizedPath(path_points);
    }
    // 2. Go through every point of the predicted obstacle trajectory.
    for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
      const auto& trajectory_point = trajectory.trajectory_point(i);
      const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);

      double trajectory_point_time = trajectory_point.relative_time();
      static constexpr double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      const double step_length = vehicle_param_.front_edge_to_center();
      auto path_len =
          std::min(FLAGS_max_trajectory_len, discretized_path.Length());
      // Go through every point of the ADC's path.
      for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
        const auto curr_adc_path_point =
            discretized_path.Evaluate(path_s + discretized_path.front().s());
        if (CheckOverlap(curr_adc_path_point, obs_box, l_buffer)) {
          // Found overlap, start searching with higher resolution
          const double backward_distance = -step_length;
          const double forward_distance = vehicle_param_.length() +
                                          vehicle_param_.width() +
                                          obs_box.length() + obs_box.width();
          const double default_min_step = 0.1;  // in meters
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);

          bool find_low = false;
          bool find_high = false;
          double low_s = std::fmax(0.0, path_s + backward_distance);
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);

          // Keep shrinking by the resolution bidirectionally until finally
          // locating the tight upper and lower bounds.
          while (low_s < high_s) {
            if (find_low && find_high) {
              break;
            }
            if (!find_low) {
              const auto& point_low = discretized_path.Evaluate(
                  low_s + discretized_path.front().s());
              if (!CheckOverlap(point_low, obs_box, l_buffer)) {
                low_s += fine_tuning_step_length;
              } else {
                find_low = true;
              }
            }
            if (!find_high) {
              const auto& point_high = discretized_path.Evaluate(
                  high_s + discretized_path.front().s());
              if (!CheckOverlap(point_high, obs_box, l_buffer)) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;
              }
            }
          }
          if (find_high && find_low) {
            lower_points->emplace_back(
                low_s - speed_bounds_config_.point_extension(),
                trajectory_point_time);
            upper_points->emplace_back(
                high_s + speed_bounds_config_.point_extension(),
                trajectory_point_time);
          }
          break;
        }
      }
    }
  }

  // Sanity checks and return.
  DCHECK_EQ(lower_points->size(), upper_points->size());
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

void STBoundaryMapper::ComputeSTBoundaryWithDecision(
    Obstacle* obstacle, const ObjectDecisionType& decision) const {
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;

  if (FLAGS_use_st_drivable_boundary &&
      obstacle->is_path_st_boundary_initialized()) {
    const auto& path_st_boundary = obstacle->path_st_boundary();
    lower_points = path_st_boundary.lower_points();
    upper_points = path_st_boundary.upper_points();
  } else {
    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return;
    }
  }

  auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

  // get characteristic_length and boundary_type.
  STBoundary::BoundaryType b_type = STBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s());
    b_type = STBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = STBoundary::CreateInstance(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = STBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = STBoundary::BoundaryType::OVERTAKE;
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);
  boundary.set_id(obstacle->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  obstacle->set_path_st_boundary(boundary);
}

bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

}  // namespace planning
}  // namespace apollo
