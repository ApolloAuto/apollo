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
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

NaviPathDecider::NaviPathDecider() : NaviTask("NaviPathDecider") {
  // TODO(all): Add your other initialization.
}

bool NaviPathDecider::Init(const PlanningConfig& config) {
  move_dest_lane_config_talbe_.clear();
  max_speed_levels_.clear();
  PlannerNaviConfig planner_navi_conf =
      config.navigation_planning_config().planner_navi_config();
  config_ = planner_navi_conf.navi_path_decider_config();
  auto move_dest_lane_config_talbe = config_.move_dest_lane_config_talbe();
  for (const auto& item : move_dest_lane_config_talbe.lateral_shift()) {
    double max_speed_level = item.max_speed();
    double max_move_dest_lane_shift_y = item.max_move_dest_lane_shift_y();
    if (move_dest_lane_config_talbe_.find(max_speed_level) ==
        move_dest_lane_config_talbe_.end()) {
      move_dest_lane_config_talbe_.emplace(
          std::make_pair(max_speed_level, max_move_dest_lane_shift_y));
      max_speed_levels_.push_back(max_speed_level);
    }
  }
  AINFO << "Maximum speeds and move to dest lane config: ";
  for (const auto& data : move_dest_lane_config_talbe_) {
    auto max_speed = data.first;
    auto max_move_dest_lane_shift_y = data.second;
    AINFO << "[max_speed : " << max_speed
          << " ,max move dest lane shift y : " << max_move_dest_lane_shift_y
          << "]";
  }
  max_keep_lane_distance_ = config_.max_keep_lane_distance();
  max_keep_lane_shift_y_ = config_.max_keep_lane_shift_y();
  min_keep_lane_offset_ = config_.min_keep_lane_offset();
  keep_lane_shift_compensation_ = config_.keep_lane_shift_compensation();
  start_plan_point_from_ = config_.start_plan_point_from();
  move_dest_lane_compensation_ = config_.move_dest_lane_compensation();

  is_init_ = obstacle_decider_.Init(config);
  return is_init_;
}

Status NaviPathDecider::Execute(Frame* frame,
                                ReferenceLineInfo* const reference_line_info) {
  NaviTask::Execute(frame, reference_line_info);
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
  start_plan_point_.set_x(vehicle_state_.x());
  start_plan_point_.set_y(vehicle_state_.y());
  start_plan_point_.set_theta(vehicle_state_.heading());
  start_plan_v_ = vehicle_state_.linear_velocity();
  start_plan_a_ = vehicle_state_.linear_acceleration();
  if (start_plan_point_from_ == 1) {
    // start plan point from planning schedule
    start_plan_point_.set_x(init_point.path_point().x());
    start_plan_point_.set_y(init_point.path_point().y());
    start_plan_point_.set_theta(init_point.path_point().theta());
    start_plan_v_ = init_point.v();
    start_plan_a_ = init_point.a();
  }

  // intercept path points from reference line
  std::vector<apollo::common::PathPoint> path_points;
  if (!GetBasicPathData(reference_line, &path_points)) {
    AERROR << "Get path points from reference line failed";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider GetBasicPathData");
  }

  // according to the position of the start plan point and the reference line,
  // the path trajectory intercepted from the reference line is shifted on the
  // y-axis to adc.
  double dest_ref_line_y = path_points[0].y();

  ADEBUG << "in current plan cycle, adc to ref line distance : "
         << dest_ref_line_y << "lane id : " << cur_reference_line_lane_id_;
  MoveToDestLane(dest_ref_line_y, &path_points);

  KeepLane(dest_ref_line_y, &path_points);

  path_data->SetReferenceLine(&(reference_line_info_->reference_line()));
  if (!path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)))) {
    AERROR << "Set path data failed.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider SetDiscretizedPath");
  }

  return Status::OK();
}

void NaviPathDecider::MoveToDestLane(
    const double dest_ref_line_y,
    std::vector<common::PathPoint>* const path_points) {
  double dest_lateral_distance = std::fabs(dest_ref_line_y);
  if (dest_lateral_distance < max_keep_lane_distance_) {
    return;
  }

  // calculate lateral shift range and theta chage ratio
  double max_shift_y = CalculateDistanceToDestLane();

  double actual_start_point_y = std::copysign(max_shift_y, dest_ref_line_y);
  // lateral shift path_points to the max_y
  double lateral_shift_value = -dest_ref_line_y + actual_start_point_y;
  // The steering wheel is more sensitive to the left than to the right and
  // requires a compensation value to the right
  lateral_shift_value =
      lateral_shift_value > 0.0
          ? (lateral_shift_value - move_dest_lane_compensation_)
          : lateral_shift_value;
  ADEBUG << "in current plan cycle move to dest lane, adc shift to dest "
            "reference line : "
         << lateral_shift_value;
  std::transform(path_points->begin(), path_points->end(), path_points->begin(),
                 [lateral_shift_value](common::PathPoint& old_path_point) {
                   common::PathPoint new_path_point = old_path_point;
                   double new_path_point_y =
                       old_path_point.y() + lateral_shift_value;
                   new_path_point.set_y(new_path_point_y);
                   return new_path_point;
                 });
}

void NaviPathDecider::KeepLane(
    const double dest_ref_line_y,
    std::vector<common::PathPoint>* const path_points) {
  double dest_lateral_distance = std::fabs(dest_ref_line_y);
  if (dest_lateral_distance <= max_keep_lane_distance_) {
    auto& reference_line = reference_line_info_->reference_line();
    auto obstacles = frame_->obstacles();
    auto* path_decision = reference_line_info_->path_decision();
    double actual_dest_point_y =
        NudgeProcess(reference_line, *path_points, obstacles, *path_decision,
                     vehicle_state_);

    double actual_dest_lateral_distance = std::fabs(actual_dest_point_y);
    double actual_shift_y = 0.0;
    if (actual_dest_lateral_distance > min_keep_lane_offset_) {
      double lateral_shift_value = 0.0;
      lateral_shift_value =
          (actual_dest_lateral_distance < max_keep_lane_shift_y_ +
                                              min_keep_lane_offset_ -
                                              keep_lane_shift_compensation_)
              ? (actual_dest_lateral_distance - min_keep_lane_offset_ +
                 keep_lane_shift_compensation_)
              : max_keep_lane_shift_y_;
      actual_shift_y = std::copysign(lateral_shift_value, actual_dest_point_y);
    }

    ADEBUG << "in current plan cycle keep lane, actual dest : "
           << actual_dest_point_y << " adc shift to dest : " << actual_shift_y;
    std::transform(
        path_points->begin(), path_points->end(), path_points->begin(),
        [actual_shift_y](common::PathPoint& old_path_point) {
          common::PathPoint new_path_point = old_path_point;
          double new_path_point_y = old_path_point.y() + actual_shift_y;
          new_path_point.set_y(new_path_point_y);
          return new_path_point;
        });
  }
}

void NaviPathDecider::RecordDebugInfo(const PathData& path_data) {
  const auto& path_points = path_data.discretized_path();
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

bool NaviPathDecider::GetBasicPathData(
    const ReferenceLine& reference_line,
    std::vector<common::PathPoint>* const path_points) {
  CHECK_NOTNULL(path_points);

  double min_path_len = config_.min_path_length();
  // get min path plan length s = v0 * t + 1 / 2.0 * a * t^2
  double path_len =
      start_plan_v_ * config_.min_look_forward_time() +
      start_plan_a_ * pow(config_.min_look_forward_time(), 2) / 2.0;
  path_len = std::max(path_len, min_path_len);

  const double reference_line_len = reference_line.Length();
  if (reference_line_len < path_len) {
    AERROR << "Reference line is too short to generate path trajectory( s = "
           << reference_line_len << ").";
    return false;
  }

  // get the start plan point project s on reference line and get the length of
  // reference line
  auto start_plan_point_project = reference_line.GetReferencePoint(
      start_plan_point_.x(), start_plan_point_.y());
  common::SLPoint sl_point;
  if (!reference_line.XYToSL(start_plan_point_project.ToPathPoint(0.0),
                             &sl_point)) {
    AERROR << "Failed to get start plan point s from reference "
              "line.";
    return false;
  }
  auto start_plan_point_project_s = sl_point.has_s() ? sl_point.s() : 0.0;

  // get basic path points form reference_line
  ADEBUG << "Basic path data len ; " << reference_line_len;
  static constexpr double KDenseSampleUnit = 0.50;
  static constexpr double KSparseSmapleUnit = 2.0;
  for (double s = start_plan_point_project_s; s < reference_line_len;
       s += ((s < path_len) ? KDenseSampleUnit : KSparseSmapleUnit)) {
    const auto& ref_point = reference_line.GetReferencePoint(s);
    auto path_point = ref_point.ToPathPoint(s - start_plan_point_project_s);
    path_points->emplace_back(path_point);
  }

  if (path_points->empty()) {
    AERROR << "path poins is empty.";
    return false;
  }

  return true;
}

bool NaviPathDecider::IsSafeChangeLane(const ReferenceLine& reference_line,
                                       const PathDecision& path_decision) {
  const auto& adc_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  Vec2d adc_position(start_plan_point_.x(), start_plan_point_.y());
  Vec2d vec_to_center(
      (adc_param.front_edge_to_center() - adc_param.back_edge_to_center()) /
          2.0,
      (adc_param.left_edge_to_center() - adc_param.right_edge_to_center()) /
          2.0);
  Vec2d adc_center(adc_position +
                   vec_to_center.rotate(start_plan_point_.theta()));
  Box2d adc_box(adc_center, start_plan_point_.theta(), adc_param.length(),
                adc_param.width());
  SLBoundary adc_sl_boundary;
  if (!reference_line.GetSLBoundary(adc_box, &adc_sl_boundary)) {
    AERROR << "Failed to get ADC boundary from box: " << adc_box.DebugString();
    return false;
  }

  for (const auto* obstacle : path_decision.obstacles().Items()) {
    const auto& sl_boundary = obstacle->PerceptionSLBoundary();
    static constexpr double kLateralShift = 6.0;
    if (sl_boundary.start_l() < -kLateralShift ||
        sl_boundary.end_l() > kLateralShift) {
      continue;
    }

    static constexpr double kSafeTime = 3.0;
    static constexpr double kForwardMinSafeDistance = 6.0;
    static constexpr double kBackwardMinSafeDistance = 8.0;

    const double kForwardSafeDistance = std::max(
        kForwardMinSafeDistance,
        ((vehicle_state_.linear_velocity() - obstacle->speed()) * kSafeTime));
    const double kBackwardSafeDistance = std::max(
        kBackwardMinSafeDistance,
        ((obstacle->speed() - vehicle_state_.linear_velocity()) * kSafeTime));
    if (sl_boundary.end_s() >
            adc_sl_boundary.start_s() - kBackwardSafeDistance &&
        sl_boundary.start_s() <
            adc_sl_boundary.end_s() + kForwardSafeDistance) {
      return false;
    }
  }

  return true;
}

double NaviPathDecider::NudgeProcess(
    const ReferenceLine& reference_line,
    const std::vector<common::PathPoint>& path_data_points,
    const std::vector<const Obstacle*>& obstacles,
    const PathDecision& path_decision,
    const common::VehicleState& vehicle_state) {
  double nudge_position_y = 0.0;

  // get nudge latteral position
  int lane_obstacles_num = 0;
  static constexpr double KNudgeEpsilon = 1e-6;
  double nudge_distance = obstacle_decider_.GetNudgeDistance(
      obstacles, reference_line, path_decision, path_data_points, vehicle_state,
      &lane_obstacles_num);
  // adjust plan start point
  if (std::fabs(nudge_distance) > KNudgeEpsilon) {
    ADEBUG << "need latteral nudge distance : " << nudge_distance;
    nudge_position_y = nudge_distance;
    last_lane_id_to_nudge_flag_[cur_reference_line_lane_id_] = true;
  } else {
    // no nudge distance but current lane has obstacles ,keepping path in
    // the last nudge path direction
    bool last_plan_has_nudge = false;
    if (last_lane_id_to_nudge_flag_.find(cur_reference_line_lane_id_) !=
        last_lane_id_to_nudge_flag_.end()) {
      last_plan_has_nudge =
          last_lane_id_to_nudge_flag_[cur_reference_line_lane_id_];
    }

    if (last_plan_has_nudge && lane_obstacles_num != 0) {
      ADEBUG << "Keepping last nudge path direction";
      nudge_position_y = vehicle_state_.y();
    } else {
      // not need nudge or not need nudge keepping
      last_lane_id_to_nudge_flag_[cur_reference_line_lane_id_] = false;
      nudge_position_y = path_data_points[0].y();
    }
  }

  return nudge_position_y;
}

double NaviPathDecider::CalculateDistanceToDestLane() {
  // match an appropriate lateral shift param from the configuration file
  // based on the current state of the vehicle state
  double move_distance = 0.0;
  double max_adc_speed =
      start_plan_v_ + start_plan_a_ * 1.0 / FLAGS_planning_loop_rate;
  auto max_speed_level_itr = std::upper_bound(
      max_speed_levels_.begin(), max_speed_levels_.end(), max_adc_speed);
  if (max_speed_level_itr != max_speed_levels_.end()) {
    auto max_speed_level = *max_speed_level_itr;
    move_distance = move_dest_lane_config_talbe_[max_speed_level];
  }

  return move_distance;
}

}  // namespace planning
}  // namespace apollo
