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

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/tasks/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/tasks/optimizers/path_decider/path_decider.h"
#include "modules/planning/tasks/optimizers/qp_piecewise_jerk_path/qp_piecewise_jerk_path_optimizer.h"
#include "modules/planning/tasks/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/tasks/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/tasks/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {

using common::ErrorCode;
using common::SLPoint;
using common::Status;
using common::TrajectoryPoint;
using common::time::Clock;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

LaneFollowStage::LaneFollowStage(const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

void LaneFollowStage::RecordObstacleDebugInfo(
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

void LaneFollowStage::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
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

Stage::StageStatus LaneFollowStage::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool has_drivable_reference_line = false;
  bool disable_low_priority_path = false;
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
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
  return has_drivable_reference_line ? StageStatus::RUNNING
                                     : StageStatus::ERROR;
}

Status LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();
  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
  auto speed_profile = SpeedProfileGenerator::GenerateInitSpeedProfile(
      planning_start_point, reference_line_info);
  if (speed_profile.empty()) {
    speed_profile =
        SpeedProfileGenerator::GenerateSpeedHotStart(planning_start_point);
    ADEBUG << "Using dummy hot start for speed vector";
  }
  *heuristic_speed_data = SpeedData(speed_profile);

  auto ret = Status::OK();

  for (auto* optimizer : task_list_) {
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
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }

  if (!ret.ok() || reference_line_info->speed_data().empty()) {
    ADEBUG << "Speed fallback.";

    *reference_line_info->mutable_speed_data() =
        SpeedProfileGenerator::GenerateFallbackSpeedProfile();
    reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }

  if (!(reference_line_info->trajectory_type() ==
            ADCTrajectory::PATH_FALLBACK ||
        reference_line_info->trajectory_type() ==
            ADCTrajectory::SPEED_FALLBACK)) {
    reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  }
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
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
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
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
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

void LaneFollowStage::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* path_data) {
  auto adc_point = EgoInfo::Instance()->start_point();
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

SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line) const {
  SLPoint sl_point;
  reference_line.XYToSL(
      {stop_decision.stop_point().x(), stop_decision.stop_point().y()},
      &sl_point);
  return sl_point;
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
