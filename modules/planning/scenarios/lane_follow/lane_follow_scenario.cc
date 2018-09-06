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
 **/

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include <fstream>
#include <limits>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {

using common::ErrorCode;
using common::SLPoint;
using common::SpeedPoint;
using common::Status;
using common::TrajectoryPoint;
using common::adapter::AdapterManager;
using common::math::Vec2d;
using common::time::Clock;

namespace {
constexpr double kPathOptimizationFallbackClost = 2e4;
constexpr double kSpeedOptimizationFallbackClost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

void LaneFollowScenario::RegisterTasks() {
  task_factory_.Register(DP_POLY_PATH_OPTIMIZER,
                         []() -> Task* { return new DpPolyPathOptimizer(); });
  task_factory_.Register(PATH_DECIDER,
                         []() -> Task* { return new PathDecider(); });
  task_factory_.Register(DP_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new DpStSpeedOptimizer(); });
  task_factory_.Register(SPEED_DECIDER,
                         []() -> Task* { return new SpeedDecider(); });
  task_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Task* {
    return new QpSplineStSpeedOptimizer();
  });
  task_factory_.Register(POLY_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new PolyStSpeedOptimizer(); });
}

bool LaneFollowScenario::Init(const PlanningConfig& config) {
  if (is_init_) {
    return true;
  }
  RegisterTasks();
  for (const auto task : config.lane_follow_scenario_config().task()) {
    tasks_.emplace_back(
        task_factory_.CreateObject(static_cast<TaskType>(task)));
    AINFO << "Created task:" << tasks_.back()->Name();
  }
  for (auto& task : tasks_) {
    if (!task->Init(config)) {
      std::string msg(
          common::util::StrCat("Init task[", task->Name(), "] failed."));
      AERROR << msg;
      return false;
    }
  }
  is_init_ = true;
  return true;
}

void LaneFollowScenario::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  auto ptr_debug = reference_line_info->mutable_debug();

  const auto path_decision = reference_line_info->path_decision();
  for (const auto path_obstacle : path_decision->path_obstacles().Items()) {
    auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
    obstacle_debug->set_id(path_obstacle->Id());
    obstacle_debug->mutable_sl_boundary()->CopyFrom(
        path_obstacle->PerceptionSLBoundary());
    const auto& decider_tags = path_obstacle->decider_tags();
    const auto& decisions = path_obstacle->decisions();
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

void LaneFollowScenario::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
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

Status LaneFollowScenario::Process(const TrajectoryPoint& planning_start_point,
                                   Frame* frame) {
  bool has_drivable_reference_line = false;
  bool disable_low_priority_path = false;
  auto status =
      Status(ErrorCode::PLANNING_ERROR, "reference line not drivable");
  for (auto& reference_line_info : frame->reference_line_info()) {
    if (disable_low_priority_path) {
      reference_line_info.SetDrivable(false);
    }
    if (!reference_line_info.IsDrivable()) {
      continue;
    }
    auto cur_status =
        PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);
    if (cur_status.ok() && reference_line_info.IsDrivable()) {
      has_drivable_reference_line = true;
      if (FLAGS_prioritize_change_lane &&
          reference_line_info.IsChangeLanePath() &&
          reference_line_info.Cost() < kStraightForwardLineCost) {
        disable_low_priority_path = true;
      }
    } else {
      reference_line_info.SetDrivable(false);
    }
  }
  return has_drivable_reference_line ? Status::OK() : status;
}

Status LaneFollowScenario::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
  auto speed_profile = speed_profile_generator_.GenerateInitSpeedProfile(
      planning_start_point, reference_line_info);
  if (speed_profile.empty()) {
    speed_profile =
        speed_profile_generator_.GenerateSpeedHotStart(planning_start_point);
    ADEBUG << "Using dummy hot start for speed vector";
  }
  heuristic_speed_data->set_speed_vector(speed_profile);

  auto ret = Status::OK();

  for (auto& optimizer : tasks_) {
    const double start_timestamp = Clock::NowInSeconds();
    ret = optimizer->Execute(frame, reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << optimizer->Name()
             << "], Error message: " << ret.error_message();
      break;
    }
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "after optimizer " << optimizer->Name() << ":"
           << reference_line_info->PathSpeedDebugString() << std::endl;
    ADEBUG << optimizer->Name() << " time spend: " << time_diff_ms << " ms.";

    RecordDebugInfo(reference_line_info, optimizer->Name(), time_diff_ms);
  }

  RecordObstacleDebugInfo(reference_line_info);

  if (reference_line_info->path_data().Empty()) {
    ADEBUG << "Path fallback.";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackClost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }

  if (!ret.ok() || reference_line_info->speed_data().Empty()) {
    ADEBUG << "Speed fallback.";

    *reference_line_info->mutable_speed_data() =
        speed_profile_generator_.GenerateFallbackSpeedProfile();
    reference_line_info->AddCost(kSpeedOptimizationFallbackClost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }

  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;
  for (const auto* path_obstacle :
       reference_line_info->path_decision()->path_obstacles().Items()) {
    if (path_obstacle->LongitudinalDecision().has_stop() &&
        path_obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(path_obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }

  for (const auto* path_obstacle :
       reference_line_info->path_decision()->path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual()) {
      continue;
    }
    if (!path_obstacle->obstacle()->IsStatic()) {
      continue;
    }
    if (path_obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl =
            GetStopSL(path_obstacle->LongitudinalDecision().stop(),
                      reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        constexpr double kRefrenceLineStaticObsCost = 1e3;
        reference_line_info->AddCost(kRefrenceLineStaticObsCost);
      }
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

void LaneFollowScenario::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  auto adc_point = EgoInfo::instance()->start_point();
  double adc_s = reference_line_info->AdcSlBoundary().end_s();
  const double max_s = 150.0;
  const double unit_s = 1.0;

  // projection of adc point onto reference line
  const auto& adc_ref_point =
      reference_line_info->reference_line().GetReferencePoint(adc_s);

  DCHECK(adc_point.has_path_point());
  const double dx = adc_point.path_point().x() - adc_ref_point.x();
  const double dy = adc_point.path_point().y() - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  for (double s = adc_s; s < max_s; s += unit_s) {
    const auto& ref_point =
        reference_line_info->reference_line().GetReferencePoint(adc_s);
    common::PathPoint path_point = common::util::MakePathPoint(
        ref_point.x() + dx, ref_point.y() + dy, 0.0, ref_point.heading(),
        ref_point.kappa(), ref_point.dkappa(), 0.0);
    path_point.set_s(s);

    path_points.push_back(std::move(path_point));
  }
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

SLPoint LaneFollowScenario::GetStopSL(
    const ObjectStop& stop_decision,
    const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  reference_line.XYToSL(
      {stop_decision.stop_point().x(), stop_decision.stop_point().y()},
      &sl_point);
  return sl_point;
}

ScenarioConfig::ScenarioType LaneFollowScenario::Transfer(
    const ScenarioConfig::ScenarioType& current_scenario,
    const common::TrajectoryPoint& ego_point, const Frame& frame) const {
  // implement here
  return ScenarioConfig::LANE_FOLLOW;
}

}  // namespace planning
}  // namespace apollo
