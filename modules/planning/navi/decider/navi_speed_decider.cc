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
 * @brief This file provides the implementation of the class "NaviSpeedDecider".
 */

#include "modules/planning/navi/decider/navi_speed_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "glog/logging.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Clamp;
using apollo::common::util::MakePathPoint;

namespace {
constexpr double kTsGraphSStep = 0.4;
constexpr size_t kFallbackSpeedPointNum = 4;
constexpr double kSpeedPointSLimit = 200.0;
constexpr double kSpeedPointTimeLimit = 50.0;
}  // namespace

NaviSpeedDecider::NaviSpeedDecider() : Task("NaviSpeedDecider") {}

bool NaviSpeedDecider::Init(const PlanningConfig& config) {
  CHECK_GT(FLAGS_planning_upper_speed_limit, 0.0);
  CHECK(config.has_navi_planner_config());
  CHECK(config.navi_planner_config().has_navi_speed_decider_config());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_preferred_accel());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_preferred_decel());
  CHECK(
      config.navi_planner_config().navi_speed_decider_config().has_max_accel());
  CHECK(
      config.navi_planner_config().navi_speed_decider_config().has_max_decel());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_preferred_jerk());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_obstacle_buffer());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_safe_distance_base());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_safe_distance_ratio());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_following_accel_ratio());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_soft_centric_accel_limit());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_hard_centric_accel_limit());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_hard_speed_limit());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_hard_accel_limit());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_enable_safe_path());

  max_speed_ = FLAGS_planning_upper_speed_limit;
  preferred_accel_ = std::abs(config.navi_planner_config()
                                  .navi_speed_decider_config()
                                  .preferred_accel());
  preferred_decel_ = std::abs(config.navi_planner_config()
                                  .navi_speed_decider_config()
                                  .preferred_decel());
  preferred_jerk_ = std::abs(config.navi_planner_config()
                                 .navi_speed_decider_config()
                                 .preferred_jerk());
  max_accel_ = std::abs(
      config.navi_planner_config().navi_speed_decider_config().max_accel());
  max_decel_ = std::abs(
      config.navi_planner_config().navi_speed_decider_config().max_decel());
  preferred_accel_ = std::min(max_accel_, preferred_accel_);
  preferred_decel_ = std::min(max_decel_, preferred_accel_);

  obstacle_buffer_ = std::abs(config.navi_planner_config()
                                  .navi_speed_decider_config()
                                  .obstacle_buffer());
  safe_distance_base_ = std::abs(config.navi_planner_config()
                                     .navi_speed_decider_config()
                                     .safe_distance_base());
  safe_distance_ratio_ = std::abs(config.navi_planner_config()
                                      .navi_speed_decider_config()
                                      .safe_distance_ratio());
  following_accel_ratio_ = std::abs(config.navi_planner_config()
                                        .navi_speed_decider_config()
                                        .following_accel_ratio());
  soft_centric_accel_limit_ = std::abs(config.navi_planner_config()
                                           .navi_speed_decider_config()
                                           .soft_centric_accel_limit());
  hard_centric_accel_limit_ = std::abs(config.navi_planner_config()
                                           .navi_speed_decider_config()
                                           .hard_centric_accel_limit());
  soft_centric_accel_limit_ =
      std::min(hard_centric_accel_limit_, soft_centric_accel_limit_);
  hard_speed_limit_ = std::abs(config.navi_planner_config()
                                   .navi_speed_decider_config()
                                   .hard_speed_limit());
  hard_accel_limit_ = std::abs(config.navi_planner_config()
                                   .navi_speed_decider_config()
                                   .hard_accel_limit());
  enable_safe_path_ = config.navi_planner_config()
                          .navi_speed_decider_config()
                          .enable_safe_path();

  return true;
}

Status NaviSpeedDecider::Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);

  // get cruise speed
  const auto& planning_target = reference_line_info_->planning_target();
  preferred_speed_ = planning_target.has_cruise_speed()
                         ? std::abs(planning_target.cruise_speed())
                         : 0.0;
  preferred_speed_ = std::min(max_speed_, preferred_speed_);

  // get the start point
  const auto& planning_start_point = frame->PlanningStartPoint();
  auto start_v = planning_start_point.has_v() ? planning_start_point.v() : 0.0;
  start_v = std::max(0.0, start_v);
  auto start_a = planning_start_point.has_a() ? planning_start_point.a() : 0.0;
  start_a = Clamp(start_a, max_accel_, -1.0 * max_decel_);
  auto start_da = 0.0;

  // get the path
  auto& discretized_path = reference_line_info_->path_data().discretized_path();
  const auto& path_points = discretized_path.path_points();

  auto ret = MakeSpeedDecision(
      start_v, start_a, start_da, path_points, frame_->obstacles(),
      [&](const std::string& id) { return frame_->Find(id); },
      reference_line_info_->mutable_speed_data());
  RecordDebugInfo(reference_line_info->speed_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }

  return ret;
}

Status NaviSpeedDecider::MakeSpeedDecision(
    double start_v, double start_a, double start_da,
    const std::vector<PathPoint>& path_points,
    const std::vector<const Obstacle*>& obstacles,
    const std::function<const Obstacle*(const std::string&)>& find_obstacle,
    SpeedData* const speed_data) {
  CHECK_NOTNULL(speed_data);
  CHECK_GE(path_points.size(), 2);

  auto start_s = path_points.front().has_s() ? path_points.front().s() : 0.0;
  auto end_s = path_points.back().has_s() ? path_points.back().s() : start_s;
  auto planning_length = end_s - start_s;

  ADEBUG << "start to make speed decision,  start_v: " << start_v
         << " start_a: " << start_a << " start_da: " << start_da
         << " start_s: " << start_s << " planning_length: " << planning_length;

  if (start_v > max_speed_) {
    AERROR << "exceeding maximum allowable speed.";
    return Status(ErrorCode::PLANNING_ERROR,
                  "exceeding maximum allowable speed.");
  }
  start_v = std::max(0.0, start_v);

  auto s_step = planning_length > kTsGraphSStep
                    ? kTsGraphSStep
                    : planning_length / kFallbackSpeedPointNum;

  // init t-s graph
  ts_graph_.Reset(s_step, planning_length, start_v, start_a, start_da);

  // add t-s constraints
  auto ret = AddPerceptionRangeConstraints();
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on range of perception failed";
    return ret;
  }

  ret = AddObstaclesConstraints(start_v, planning_length, path_points,
                                obstacles, find_obstacle);
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on obstacles failed";
    return ret;
  }

  ret = AddCentricAccelerationConstraints(path_points);
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on centric acceleration failed";
    return ret;
  }

  ret = AddConfiguredConstraints();
  if (ret != Status::OK()) {
    AERROR << "Add t-s constraints base on configs failed";
    return ret;
  }

  // create speed-points
  std::vector<NaviSpeedTsPoint> ts_points;
  ret = ts_graph_.Solve(&ts_points);
  if (ret != Status::OK()) {
    AERROR << "Solve speed points failed";
    return ret;
  }

  speed_data->Clear();
  for (auto& ts_point : ts_points) {
    if (ts_point.s > kSpeedPointSLimit || ts_point.t > kSpeedPointTimeLimit)
      break;

    if (ts_point.v > hard_speed_limit_) {
      AERROR << "The v: " << ts_point.v << " of point with s: " << ts_point.s
             << " and t: " << ts_point.t << "is greater than hard_speed_limit "
             << hard_speed_limit_;
      ts_point.v = hard_speed_limit_;
    }

    if (ts_point.a > hard_accel_limit_) {
      AERROR << "The a: " << ts_point.a << " of point with s: " << ts_point.s
             << " and t: " << ts_point.t << "is greater than hard_accel_limit "
             << hard_accel_limit_;
      ts_point.a = hard_accel_limit_;
    }

    speed_data->AppendSpeedPoint(ts_point.s + start_s, ts_point.t, ts_point.v,
                                 ts_point.a, ts_point.da);
  }

  const auto& speed_vector = speed_data->speed_vector();
  if (speed_vector.size() == 1) {
    const auto& prev = speed_vector.back();
    speed_data->AppendSpeedPoint(prev.s(), prev.t() + 1.0, 0.0, 0.0, 0.0);
  }

  return Status::OK();
}

Status NaviSpeedDecider::AddPerceptionRangeConstraints() {
  // TODO(all):
  return Status::OK();
}

Status NaviSpeedDecider::AddObstaclesConstraints(
    double vehicle_speed, double path_length,
    const std::vector<PathPoint>& path_points,
    const std::vector<const Obstacle*>& obstacles,
    const std::function<const Obstacle*(const std::string&)>& find_obstacle) {
  const auto& vehicle_config = VehicleConfigHelper::instance()->GetConfig();
  auto front_edge_to_center =
      vehicle_config.vehicle_param().front_edge_to_center();
  auto get_obstacle_distance = [&](double d) -> double {
    return std::max(0.0, d - front_edge_to_center - obstacle_buffer_);
  };
  auto get_safe_distance = [&](double v) -> double {
    return safe_distance_ratio_ * v + safe_distance_base_ +
           front_edge_to_center + obstacle_buffer_;
  };

  // add obstacles from perception
  obstacle_decider_.GetUnsafeObstaclesInfo(path_points, obstacles);
  for (const auto& info : obstacle_decider_.UnsafeObstacles()) {
    const auto& id = std::get<0>(info);
    const auto* obstacle = find_obstacle(id);
    if (obstacle != nullptr) {
      auto s = std::get<1>(info);
      auto obstacle_distance = get_obstacle_distance(s);
      auto rel_speed = std::get<2>(info);
      auto obstacle_speed = std::max(rel_speed + vehicle_speed, 0.0);
      auto safe_distance = get_safe_distance(obstacle_speed);
      ADEBUG << "obstacle with id: " << id << " distance: " << obstacle_distance
             << " speed: " << obstacle_speed
             << " safe_distance: " << safe_distance;

      ts_graph_.UpdateObstacleConstraints(obstacle_distance, safe_distance,
                                          following_accel_ratio_,
                                          obstacle_speed, preferred_speed_);
    }
  }

  // the end of path just as an obstacle
  if (enable_safe_path_) {
    auto obstacle_distance = get_obstacle_distance(path_length);
    auto safe_distance = get_safe_distance(0.0);
    ts_graph_.UpdateObstacleConstraints(obstacle_distance, safe_distance,
                                        following_accel_ratio_, 0.0,
                                        preferred_speed_);
  }

  return Status::OK();
}

Status AddTrafficDecisionConstraints() {
  // TODO(all):
  return Status::OK();
}

Status NaviSpeedDecider::AddCentricAccelerationConstraints(
    const std::vector<PathPoint>& path_points) {
  if (path_points.size() < 2) {
    AERROR << "Too few path points";
    return Status(ErrorCode::PLANNING_ERROR, "too few path points.");
  }

  double max_kappa = 0.0;
  double max_kappa_v = std::numeric_limits<double>::max();
  double max_kappa_s = 0.0;
  const auto bs = path_points[0].has_s() ? path_points[0].s() : 0.0;
  for (size_t i = 1; i < path_points.size(); i++) {
    const auto& prev = path_points[i - 1];
    const auto& cur = path_points[i];
    auto start_s = prev.has_s() ? prev.s() - bs : 0.0;
    start_s = std::max(0.0, start_s);
    auto end_s = cur.has_s() ? cur.s() - bs : 0.0;
    end_s = std::max(0.0, end_s);
    auto start_k = prev.has_kappa() ? prev.kappa() : 0.0;
    auto end_k = cur.has_kappa() ? cur.kappa() : 0.0;
    auto kappa = std::abs((start_k + end_k) / 2.0);
    auto v_preffered = std::min(std::sqrt(soft_centric_accel_limit_ / kappa),
                                std::numeric_limits<double>::max());
    auto v_max = std::min(std::sqrt(hard_centric_accel_limit_ / kappa),
                          std::numeric_limits<double>::max());

    NaviSpeedTsConstraints constraints;
    constraints.v_max = v_max;
    constraints.v_preffered = v_preffered;
    ts_graph_.UpdateRangeConstraints(start_s, end_s, constraints);

    if (kappa > max_kappa) {
      max_kappa = kappa;
      max_kappa_v = v_max;
      max_kappa_s = start_s;
    }
  }

  ADEBUG << "add speed limit for centric acceleration ｗith kappa: "
         << max_kappa << " v_max: " << max_kappa_v << " s: " << max_kappa_s;

  return Status::OK();
}

Status NaviSpeedDecider::AddConfiguredConstraints() {
  NaviSpeedTsConstraints constraints;
  constraints.v_max = max_speed_;
  constraints.v_preffered = preferred_speed_;
  constraints.a_max = max_accel_;
  constraints.a_preffered = preferred_accel_;
  constraints.b_max = max_decel_;
  constraints.b_preffered = preferred_decel_;
  constraints.da_preffered = preferred_jerk_;
  ts_graph_.UpdateConstraints(constraints);

  return Status::OK();
}

void NaviSpeedDecider::RecordDebugInfo(const SpeedData& speed_data) {
  auto* debug = reference_line_info_->mutable_debug();
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
  ptr_speed_plan->set_name(Name());
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
}

}  // namespace planning
}  // namespace apollo
