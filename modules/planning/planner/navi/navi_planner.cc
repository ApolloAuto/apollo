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
 * @brief This file provides the implementation of the class "NaviPlanner".
 */

#include "modules/planning/planner/navi/navi_planner.h"

#include <utility>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/navi/decider/navi_obstacle_decider.h"
#include "modules/planning/navi/decider/navi_path_decider.h"
#include "modules/planning/navi/decider/navi_speed_decider.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::common::util::PointFactory;

namespace {
constexpr uint32_t KDestLanePriority = 0;
constexpr double kPathOptimizationFallbackClost = 2e4;
constexpr double kSpeedOptimizationFallbackClost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

void NaviPlanner::RegisterTasks() {
  task_factory_.Register(TaskConfig::NAVI_PATH_DECIDER,
                         []() -> NaviTask* { return new NaviPathDecider(); });
  task_factory_.Register(TaskConfig::NAVI_SPEED_DECIDER,
                         []() -> NaviTask* { return new NaviSpeedDecider(); });
}

Status NaviPlanner::Init(const PlanningConfig& config) {
  // NaviPlanner is only used in navigation mode based on the real-time relative
  // map.
  if (!FLAGS_use_navigation_mode) {
    std::string msg = "NaviPlanner is only used in navigation mode.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  AINFO << "In NaviPlanner::Init()";
  RegisterTasks();
  PlannerNaviConfig planner_conf =
      config.navigation_planning_config().planner_navi_config();
  for (const auto task : planner_conf.task()) {
    tasks_.emplace_back(
        task_factory_.CreateObject(static_cast<TaskConfig::TaskType>(task)));
    AINFO << "Created task:" << tasks_.back()->Name();
  }
  for (auto& task : tasks_) {
    if (!task->Init(config)) {
      std::string msg(absl::StrCat("Init task[", task->Name(), "] failed."));
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

Status NaviPlanner::Plan(const TrajectoryPoint& planning_init_point,
                         Frame* frame, ADCTrajectory* ptr_computed_trajectory) {
  // NaviPlanner is only used in navigation mode based on the real-time relative
  // map.
  if (!FLAGS_use_navigation_mode) {
    std::string msg = "NaviPlanner is only used in navigation mode.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  size_t success_line_count = 0;
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    uint32_t priority = reference_line_info.GetPriority();
    reference_line_info.SetCost(priority * kStraightForwardLineCost);
    if (priority != KDestLanePriority) {
      reference_line_info.SetDrivable(false);
      continue;
    }
    auto status =
        PlanOnReferenceLine(planning_init_point, frame, &reference_line_info);

    if (status.ok() && reference_line_info.IsDrivable()) {
      success_line_count += 1;
    } else {
      reference_line_info.SetDrivable(false);
      AERROR << "Failed to plan on reference line  "
             << reference_line_info.Lanes().Id();
    }
    ADEBUG << "ref line info: " << reference_line_info.Lanes().Id()
           << " priority : " << reference_line_info.GetPriority()
           << " cost : " << reference_line_info.Cost()
           << " driveable : " << reference_line_info.IsDrivable();
  }

  if (success_line_count > 0) {
    return Status::OK();
  }

  return Status(ErrorCode::PLANNING_ERROR,
                "Failed to plan on any reference line.");
}

Status NaviPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath() &&
      reference_line_info->IsNeighborLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  ADEBUG << "planning start point:" << planning_init_point.DebugString();
  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
  auto speed_profile =
      GenerateInitSpeedProfile(planning_init_point, reference_line_info);
  if (speed_profile.empty()) {
    speed_profile = GenerateSpeedHotStart(planning_init_point);
    ADEBUG << "Using dummy hot start for speed vector";
  }
  *heuristic_speed_data = SpeedData(speed_profile);

  auto ret = Status::OK();

  for (auto& task : tasks_) {
    const double start_timestamp = Clock::NowInSeconds();
    ret = task->Execute(frame, reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret.error_message();
      break;
    }
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "after task " << task->Name() << ":"
           << reference_line_info->PathSpeedDebugString();
    ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";

    RecordDebugInfo(reference_line_info, task->Name(), time_diff_ms);
  }

  RecordObstacleDebugInfo(reference_line_info);

  if (reference_line_info->path_data().Empty()) {
    ADEBUG << "Path fallback.";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackClost);
  }

  if (!ret.ok() || reference_line_info->speed_data().empty()) {
    ADEBUG << "Speed fallback.";
    GenerateFallbackSpeedProfile(reference_line_info->mutable_speed_data());
    reference_line_info->AddCost(kSpeedOptimizationFallbackClost);
  }

  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_init_point.relative_time(),
          planning_init_point.path_point().s(), &trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop()) {
      static constexpr double kRefrenceLineStaticObsCost = 1e3;
      reference_line_info->AddCost(kRefrenceLineStaticObsCost);
    }
  }

  if (FLAGS_enable_trajectory_check) {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      std::string msg("Current planning trajectory is not valid.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  return Status::OK();
}

void NaviPlanner::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  auto ptr_debug = reference_line_info->mutable_debug();

  const auto path_decision = reference_line_info->path_decision();
  for (const auto obstacle : path_decision->obstacles().Items()) {
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(obstacle->Id());
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        obstacle->PerceptionSLBoundary());
    const auto& decider_tags = obstacle->decider_tags();
    const auto& decisions = obstacle->decisions();
    if (decider_tags.size() != decisions.size()) {
      AERROR << "decider_tags size: " << decider_tags.size()
             << " different from decisions size:" << decisions.size();
    }
    for (size_t i = 0; i < decider_tags.size(); ++i) {
      auto decision_tag = obstacle_debug->add_decision_tag();
      decision_tag->set_decider_tag(decider_tags[i]);
      decision_tag->mutable_decision()->CopyFrom(decisions[i]);
    }
  }
}

void NaviPlanner::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                                  const std::string& name,
                                  const double time_diff_ms) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";
    return;
  }

  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  auto ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
}

std::vector<SpeedPoint> NaviPlanner::GenerateInitSpeedProfile(
    const TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info) {
  std::vector<SpeedPoint> speed_profile;
  const auto* last_frame = FrameHistory::Instance()->Latest();
  if (!last_frame) {
    AWARN << "last frame is empty";
    return speed_profile;
  }
  const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();
  if (!last_reference_line_info) {
    ADEBUG << "last reference line info is empty";
    return speed_profile;
  }
  if (!reference_line_info->IsStartFrom(*last_reference_line_info)) {
    ADEBUG << "Current reference line is not started previous drived line";
    return speed_profile;
  }
  const auto& last_speed_data = last_reference_line_info->speed_data();

  if (!last_speed_data.empty()) {
    const auto& last_init_point = last_frame->PlanningStartPoint().path_point();
    Vec2d last_xy_point(last_init_point.x(), last_init_point.y());
    SLPoint last_sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(last_xy_point,
                                                           &last_sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";
    }

    Vec2d xy_point(planning_init_point.path_point().x(),
                   planning_init_point.path_point().y());
    SLPoint sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(xy_point,
                                                           &sl_point)) {
      AERROR << "Fail to transfer xy to sl when init speed profile";
    }

    double s_diff = sl_point.s() - last_sl_point.s();
    double start_time = 0.0;
    double start_s = 0.0;
    bool is_updated_start = false;
    for (const auto& speed_point : last_speed_data) {
      if (speed_point.s() < s_diff) {
        continue;
      }
      if (!is_updated_start) {
        start_time = speed_point.t();
        start_s = speed_point.s();
        is_updated_start = true;
      }
      speed_profile.push_back(PointFactory::ToSpeedPoint(
          speed_point.s() - start_s, speed_point.t() - start_time,
          speed_point.v(), speed_point.a(), speed_point.da()));
    }
  }
  return speed_profile;
}

// This is a dummy simple hot start, need refine later
std::vector<SpeedPoint> NaviPlanner::GenerateSpeedHotStart(
    const TrajectoryPoint& planning_init_point) {
  std::vector<SpeedPoint> hot_start_speed_profile;
  double s = 0.0;
  double t = 0.0;
  double v = common::math::Clamp(planning_init_point.v(), 5.0,
                                 FLAGS_planning_upper_speed_limit);
  while (t < FLAGS_trajectory_time_length) {
    hot_start_speed_profile.push_back(PointFactory::ToSpeedPoint(s, t, v));
    t += FLAGS_trajectory_time_min_interval;
    s += v * FLAGS_trajectory_time_min_interval;
  }
  return hot_start_speed_profile;
}

void NaviPlanner::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  auto adc_point = EgoInfo::Instance()->start_point();
  double adc_s = reference_line_info->AdcSlBoundary().end_s();
  const double max_s = 150.0;
  const double unit_s = 1.0;

  // projection of adc point onto reference line
  const auto& adc_ref_point =
      reference_line_info->reference_line().GetReferencePoint(0.5 * adc_s);

  DCHECK(adc_point.has_path_point());
  const double dx = adc_point.path_point().x() - adc_ref_point.x();
  const double dy = adc_point.path_point().y() - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point =
        reference_line_info->reference_line().GetReferencePoint(s);
    path_points.push_back(PointFactory::ToPathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, s, ref_point.heading(),
        ref_point.kappa(), ref_point.dkappa()));
  }
  path_data->SetReferenceLine(&(reference_line_info->reference_line()));
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

void NaviPlanner::GenerateFallbackSpeedProfile(SpeedData* speed_data) {
  const auto& start_point = EgoInfo::Instance()->start_point();
  *speed_data =
      GenerateStopProfileFromPolynomial(start_point.v(), start_point.a());
  if (speed_data->empty()) {
    *speed_data = GenerateStopProfile(start_point.v(), start_point.a());
  }
}

SpeedData NaviPlanner::GenerateStopProfile(const double init_speed,
                                           const double init_acc) const {
  AERROR << "Slowing down the car.";
  SpeedData speed_data;

  const double kFixedJerk = -1.0;
  const double first_point_acc = std::fmin(0.0, init_acc);

  const double max_t = 3.0;
  const double unit_t = 0.02;

  double pre_s = 0.0;
  const double t_mid =
      (FLAGS_slowdown_profile_deceleration - first_point_acc) / kFixedJerk;
  const double s_mid = init_speed * t_mid +
                       0.5 * first_point_acc * t_mid * t_mid +
                       1.0 / 6.0 * kFixedJerk * t_mid * t_mid * t_mid;
  const double v_mid =
      init_speed + first_point_acc * t_mid + 0.5 * kFixedJerk * t_mid * t_mid;

  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    if (t <= t_mid) {
      s = std::fmax(pre_s, init_speed * t + 0.5 * first_point_acc * t * t +
                               1.0 / 6.0 * kFixedJerk * t * t * t);
      v = std::fmax(
          0.0, init_speed + first_point_acc * t + 0.5 * kFixedJerk * t * t);
      const double a = first_point_acc + kFixedJerk * t;
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      pre_s = s;
    } else {
      s = std::fmax(pre_s, s_mid + v_mid * (t - t_mid) +
                               0.5 * FLAGS_slowdown_profile_deceleration *
                                   (t - t_mid) * (t - t_mid));
      v = std::fmax(0.0,
                    v_mid + (t - t_mid) * FLAGS_slowdown_profile_deceleration);
      speed_data.AppendSpeedPoint(s, t, v, FLAGS_slowdown_profile_deceleration,
                                  0.0);
    }
    pre_s = s;
  }
  return speed_data;
}

SpeedData NaviPlanner::GenerateStopProfileFromPolynomial(
    const double init_speed, const double init_acc) const {
  AERROR << "Slowing down the car with polynomial.";
  static constexpr double kMaxT = 4.0;
  for (double t = 2.0; t <= kMaxT; t += 0.5) {
    for (double s = 0.0; s < 50.0; s += 1.0) {
      QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, s, 0.0, 0.0, t);
      if (!IsValidProfile(curve)) {
        continue;
      }
      static constexpr double kUnitT = 0.02;
      SpeedData speed_data;
      for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT) {
        const double curve_s = curve.Evaluate(0, curve_t);
        const double curve_v = curve.Evaluate(1, curve_t);
        const double curve_a = curve.Evaluate(2, curve_t);
        const double curve_da = curve.Evaluate(3, curve_t);
        speed_data.AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a,
                                    curve_da);
      }
      return speed_data;
    }
  }
  return SpeedData();
}

bool NaviPlanner::IsValidProfile(const QuinticPolynomialCurve1d& curve) const {
  for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength();
       evaluate_t += 0.2) {
    const double v = curve.Evaluate(1, evaluate_t);
    const double a = curve.Evaluate(2, evaluate_t);
    static constexpr double kEpsilon = 1e-3;
    if (v < -kEpsilon || a < -5.0) {
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
