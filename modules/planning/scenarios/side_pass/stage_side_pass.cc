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

#include "modules/planning/scenarios/side_pass/stage_side_pass.h"

#include <utility>
#include <vector>

#include "modules/common/time/time.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using common::Status;
using common::TrajectoryPoint;
using common::time::Clock;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

StageSidePass::StageSidePass(const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

Stage::StageStatus StageSidePass::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  if (frame->mutable_reference_line_info()->empty()) {
    AERROR << "No reference line available for side pass stage";
    return StageStatus::ERROR;
  }
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  if (reference_line_info.IsChangeLanePath()) {
    AERROR << "Cannot perform side pass on a change-lane reference line!";
    return StageStatus::ERROR;
  }

  auto tasks_status =
      ExecuteTasks(planning_init_point, frame, &reference_line_info);

  if (tasks_status != Status::OK()) {
    auto fallback_status = PlanFallbackTrajectory(planning_init_point, frame,
                                                  &reference_line_info);

    if (fallback_status != Status::OK()) {
      AERROR << "computing fallback trajectory failed";
      return Stage::StageStatus::ERROR;
    }
  }

  return Stage::StageStatus::RUNNING;
}

Status StageSidePass::ExecuteTasks(const TrajectoryPoint& planning_start_point,
                                   Frame* frame,
                                   ReferenceLineInfo* reference_line_info) {
  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
  auto speed_profile = SpeedProfileGenerator::GenerateInitSpeedProfile(
      planning_start_point, reference_line_info);
  if (speed_profile.empty()) {
    speed_profile =
        SpeedProfileGenerator::GenerateSpeedHotStart(planning_start_point);
    ADEBUG << "Using dummy hot start for speed vector";
  }
  *heuristic_speed_data = SpeedData(speed_profile);

  for (auto* ptr_task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();
    auto task_status = ptr_task->Execute(frame, reference_line_info);
    if (!task_status.ok()) {
      AERROR << "Failed to run tasks[" << ptr_task->Name()
             << "], Error message: " << task_status.error_message();
      return Status(common::ErrorCode::PLANNING_ERROR);
    }
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000.0;

    ADEBUG << "after optimizer " << ptr_task->Name() << ":"
           << reference_line_info->PathSpeedDebugString() << std::endl;
    ADEBUG << ptr_task->Name() << " time spend: " << time_diff_ms << " ms.";
  }

  if (reference_line_info->path_data().Empty() ||
      reference_line_info->speed_data().empty()) {
    AERROR << "Unexpected path or speed optimizer failure.";
    return Status(common::ErrorCode::PLANNING_ERROR);
  }

  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    AERROR << "Fail to aggregate planning trajectory.";
    return Status(common::ErrorCode::PLANNING_ERROR);
  }
  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);

  return Status::OK();
}

// TODO(all): merge the fallback strategy from all scenarios
Status StageSidePass::PlanFallbackTrajectory(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  // path and speed fall back
  if (reference_line_info->path_data().Empty()) {
    AERROR << "Path fallback due to algorithm failure";
    GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }

  AERROR << "Speed fallback due to algorithm failure";
  // TODO(Jinyun) calculate front clear distance for fixed distance fallback
  const double stop_distance =
      reference_line_info->path_data().discretized_path().Length();
  *reference_line_info->mutable_speed_data() =
      SpeedProfileGenerator::GenerateFallbackSpeedProfileWithStopDistance(
          stop_distance);
  reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
  reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);

  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    AERROR << "Fail to aggregate planning trajectory.";
    return Status(common::ErrorCode::PLANNING_ERROR);
  }
  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  return Status::OK();
}

void StageSidePass::GenerateFallbackPathProfile(
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

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
