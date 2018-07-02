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
 * @brief This file provides the implementation of the class "NaviPathDecider".
 */

#include "modules/planning/navi/decider/navi_path_decider.h"

#include <algorithm>
#include <limits>

#include "glog/logging.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/navi/decider/navi_obstacle_decider.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

NaviPathDecider::NaviPathDecider() : Task("NaviPathDecider") {
  // TODO(all): Add your other initialization.
}

bool NaviPathDecider::Init(const PlanningConfig& config) {
  config_ = config.navi_planner_config().navi_path_decider_config();
  is_init_ = true;
  return true;
}

Status NaviPathDecider::Execute(Frame* frame,
                                ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  vehicle_state_ = frame->vehicle_state();
  cur_reference_line_lane_id_ = reference_line_info->Lanes().Id();
  auto ret = Process(reference_line_info->reference_line(),
                     frame->PlanningStartPoint(), frame->obstacles(),
                     reference_line_info->path_decision(),
                     reference_line_info->mutable_path_data());
  RecordDebugInfo(reference_line_info->path_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}

apollo::common::Status NaviPathDecider::Process(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point,
    const std::vector<const Obstacle*>& obstacles,
    PathDecision* const path_decision, PathData* const path_data) {
  CHECK_NOTNULL(path_decision);
  CHECK_NOTNULL(path_data);

  // get min path plan lenth
  size_t path_len = static_cast<size_t>(
      std::ceil(init_point.v() * config_.min_look_forward_time()));
  size_t min_path_len = config_.min_path_length();
  path_len = path_len > min_path_len ? path_len : min_path_len;

  // intercept path points from reference line
  std::vector<apollo::common::PathPoint> path_points;
  if (!GetLocalPath(reference_line, init_point, &path_points) ||
      path_points.size() < path_len) {
    AERROR << "Get path points from reference line failed.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider GetLocalPath");
  }
  // according to the position of the start plan point and the reference line,
  // the path trajectory intercepted from the reference line is shifted on the
  // y-axis.
  double init_local_path_y = path_points[0].y();
  double max_lateral_distance = config_.max_lateral_distance();
  if (std::fabs(init_local_path_y) > max_lateral_distance) {
    AERROR << "The reference line is too far from the car to plan.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider reference is too far from the car");
  }

  // get the projection point coordinates of the reference line from the actual
  // position of the adc
  auto adc_project_point =
      reference_line.GetReferencePoint(vehicle_state_.x(), vehicle_state_.y());
  auto& adc_lane_way_points = adc_project_point.lane_waypoints();
  if (adc_lane_way_points.empty()) {
    AERROR << "Failed to get adc lane way points from reference line.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider failed to get adc lane way points ");
  }
  double project_point_s = adc_lane_way_points[0].s;
  auto adc_project_path_point =
      reference_line.GetReferencePoint(project_point_s)
          .ToPathPoint(project_point_s);
  double cur_ref_line_start_y = adc_project_point.y();

  // get y-coordinate of the target start path plan point
  double target_start_path_point_y = init_local_path_y;
  if (reference_line_info_->IsChangeLanePath() &&
      reference_line_info_->IsNeighborLanePath()) {
    ADEBUG << "change lane path plan";
    // check whether it is safe to change lanes
    if (!IsSafeChangeLane(reference_line, *path_decision)) {
      AERROR << "Change lane failed ,because it is not safe to change lane";
      return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                    "NaviPathDecider It is not safe to change lane");
    }
  } else {
    ADEBUG << "common lane path plan";
    // to adjust target start point following the result of nudge processing
    if (FLAGS_enable_nudge_decision) {
      // get the min lane width from
      double min_lane_width = std::numeric_limits<double>::max();
      constexpr double KNudgeZero = 1e-6;
      std::for_each(path_points.begin(), path_points.end(),
                    [&min_lane_width, reference_line](
                        const apollo::common::PathPoint& path_point) {
                      double lane_left_width = 0.0;
                      double lane_right_width = 0.0;
                      bool bRet = reference_line.GetLaneWidth(
                          path_point.s(), &lane_left_width, &lane_right_width);
                      if (bRet) {
                        double lane_width = lane_left_width + lane_right_width;
                        if (lane_width < min_lane_width) {
                          min_lane_width = lane_width;
                        }
                      }
                    });

      // get nudge latteral position
      NaviObstacleDecider obstacle_decider;
      double nudge_distance = obstacle_decider.GetNudgeDistance(
          obstacles, path_points, min_lane_width);
      // adjust plan start point
      if (std::fabs(nudge_distance) > KNudgeZero) {
        target_start_path_point_y = nudge_distance;
      }
    }
  }

  // caculate the y-coordinate of the actual start path plan point
  double start_point_y =
      SmoothInitY(cur_ref_line_start_y, target_start_path_point_y);

  // shift trajectory intercepted from the reference line
  double shift_distance_y = start_point_y - init_local_path_y;
  ShiftY(shift_distance_y, &path_points);

  // calculate the value of the path trajectory later
  reference_line_info_->AddCost(0.0);

  DiscretizedPath discretized_path(path_points);
  path_data->SetReferenceLine(&(reference_line_info_->reference_line()));
  if (!path_data->SetDiscretizedPath(discretized_path)) {
    AERROR << "Set path data failed.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider SetDiscretizedPath");
  }
  last_lane_id_to_start_y_[cur_reference_line_lane_id_] = cur_ref_line_start_y;

  return Status::OK();
}

void NaviPathDecider::RecordDebugInfo(const PathData& path_data) {
  const auto& path_points = path_data.discretized_path().path_points();
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

bool NaviPathDecider::GetLocalPath(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point,
    std::vector<common::PathPoint>* const path_points) {
  CHECK_NOTNULL(path_points);
  if (reference_line.reference_points().size() < 10) {
    AERROR
        << "Reference line points is not enough to generate path trajectory.";
    return false;
  }

  // find the projection reference point of the car on the reference line
  double start_plan_point_x = init_point.path_point().x();
  double start_plan_point_y = init_point.path_point().y();
  auto project_point =
      reference_line.GetReferencePoint(start_plan_point_x, start_plan_point_y);
  double reference_line_len = reference_line.Length();
  auto& lane_way_points = project_point.lane_waypoints();
  if (lane_way_points.empty()) {
    AERROR << "Failed to get start plan point lane way points from reference "
              "line.";
    return false;
  }

  // get path points form reference_line
  double project_point_s = lane_way_points[0].s;
  size_t reference_point_num =
      static_cast<size_t>(std::floor(reference_line_len - project_point_s)) + 1;
  for (size_t i = 0; i < reference_point_num; ++i) {
    double s = i + project_point_s;
    auto point = reference_line.GetReferencePoint(s);
    path_points->emplace_back(point.ToPathPoint(s));
  }

  return true;
}

void NaviPathDecider::ShiftY(
    const double shift_distance,
    std::vector<common::PathPoint>* const path_points) {
  CHECK_NOTNULL(path_points);
  std::transform(path_points->begin(), path_points->end(), path_points->begin(),
                 [shift_distance](common::PathPoint& old_path_point) {
                   common::PathPoint new_path_point = old_path_point;
                   new_path_point.set_y(old_path_point.y() + shift_distance);
                   return new_path_point;
                 });
}

double NaviPathDecider::SmoothInitY(const double actual_ref_init_y,
                                    const double target_path_init_y) {
  constexpr double KShiftZero = 1e-6;
  const double max_init_y = config_.max_smooth_init_y();
  const double min_init_y = config_.min_smooth_init_y();
  double delta_shift = config_.lateral_shift_delta();
  double cur_adc_shift = 0.0;

  // need to adjust in lateral
  if (std::fabs(target_path_init_y) > min_init_y) {
    double last_adc_lateral_shift =
        last_lane_id_to_start_y_[cur_reference_line_lane_id_] -
        actual_ref_init_y;

    // last shift distacne was zero,initialize a suitable amount of shift
    if (std::fabs(last_adc_lateral_shift) < KShiftZero) {
      if (target_path_init_y > 0) {
        cur_adc_shift = min_init_y;
      } else {
        cur_adc_shift = -min_init_y;
      }
    } else {  // the last shift distance was not zero
      // The direction of the last lateral shift is the same as the direction
      // that needs to be shifted this time
      if (target_path_init_y * last_adc_lateral_shift > 0) {
        if (last_adc_lateral_shift > 0) {
          cur_adc_shift = last_adc_lateral_shift + delta_shift;
          cur_adc_shift =
              cur_adc_shift < max_init_y ? cur_adc_shift : max_init_y;
        } else {
          cur_adc_shift = last_adc_lateral_shift - delta_shift;
          cur_adc_shift =
              cur_adc_shift > -max_init_y ? cur_adc_shift : -max_init_y;
        }
      } else {
        // The direction of the last lateral shift is opposite to the
        // direction that needs to be shifted this time
        if (target_path_init_y > 0) {
          cur_adc_shift = min_init_y;
        } else {
          cur_adc_shift = -min_init_y;
        }
      }
    }

    // if shift exceeded target
    if (std::fabs(target_path_init_y) < std::fabs(cur_adc_shift)) {
      cur_adc_shift = target_path_init_y;
    }
  }

  return cur_adc_shift;
}

bool NaviPathDecider::IsSafeChangeLane(const ReferenceLine& reference_line,
                                       const PathDecision& path_decision) {
  const auto& adc_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  common::PathPoint start_plan_point =
      frame_->PlanningStartPoint().path_point();
  Vec2d adc_position(start_plan_point.x(), start_plan_point.y());
  Vec2d vec_to_center(
      (adc_param.front_edge_to_center() - adc_param.back_edge_to_center()) /
          2.0,
      (adc_param.left_edge_to_center() - adc_param.right_edge_to_center()) /
          2.0);
  Vec2d adc_center(adc_position +
                   vec_to_center.rotate(start_plan_point.theta()));
  Box2d adc_box(adc_center, start_plan_point.theta(), adc_param.length(),
                adc_param.width());
  SLBoundary adc_sl_boundary;
  if (!reference_line.GetSLBoundary(adc_box, &adc_sl_boundary)) {
    AERROR << "Failed to get ADC boundary from box: " << adc_box.DebugString();
    return false;
  }

  for (const auto* path_obstacle : path_decision.path_obstacles().Items()) {
    const auto& sl_boundary = path_obstacle->PerceptionSLBoundary();
    constexpr double kLateralShift = 6.0;
    if (sl_boundary.start_l() < -kLateralShift ||
        sl_boundary.end_l() > kLateralShift) {
      continue;
    }

    constexpr double kSafeTime = 3.0;
    constexpr double kForwardMinSafeDistance = 6.0;
    constexpr double kBackwardMinSafeDistance = 8.0;

    const double kForwardSafeDistance = std::max(
        kForwardMinSafeDistance, ((vehicle_state_.linear_velocity() -
                                   path_obstacle->obstacle()->Speed()) *
                                  kSafeTime));
    const float kBackwardSafeDistance = std::max(
        kBackwardMinSafeDistance, ((path_obstacle->obstacle()->Speed() -
                                    vehicle_state_.linear_velocity()) *
                                   kSafeTime));
    if (sl_boundary.end_s() >
            adc_sl_boundary.start_s() - kBackwardSafeDistance &&
        sl_boundary.start_s() <
            adc_sl_boundary.end_s() + kForwardSafeDistance) {
      return false;
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
