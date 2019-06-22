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

#include "modules/planning/scenarios/park/pull_over/stage_approach.h"

#include <string>
#include <vector>

#include "cyber/common/log.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace pull_over {

using common::TrajectoryPoint;

PullOverStageApproach::PullOverStageApproach(
    const ScenarioConfig::StageConfig& config)
    : Stage(config) {}

Stage::StageStatus PullOverStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "PullOverStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  scenario::util::PullOverStatus status =
      scenario::util::CheckADCPullOver(reference_line_info, scenario_config_);

  if (status == scenario::util::PASS_DESTINATION ||
      status == scenario::util::PARK_COMPLETE) {
    return FinishStage(true);
  } else if (status == scenario::util::PARK_FAIL) {
    return FinishStage(false);
  }

  // chek path_data to fail sooner
  bool path_fail = false;
  const auto& candidate_path_data = reference_line_info.GetCandidatePathData();
  if (!candidate_path_data.empty()) {
    for (const auto& path_data : candidate_path_data) {
      if (path_data.path_label().find("pullover") == std::string::npos) {
        break;
      }

      for (size_t i = path_data.discretized_path().size() - 1; i >= 0; --i) {
        if (path_data.frenet_frame_path().back().s() -
                path_data.frenet_frame_path()[i].s() <
            kNumExtraTailBoundPoint * kPathBoundsDeciderResolution) {
          continue;
        }
        // check the last adc_position planned
        const auto& path_point = path_data.discretized_path()[i];
        scenario::util::PullOverStatus status =
            scenario::util::CheckADCPullOverPathPoint(
                reference_line_info, scenario_config_, path_point);
        if (status == scenario::util::PARK_FAIL) {
          path_fail = true;
        }
        break;
      }
    }
  }

  // add a stop fence for adc to pause at a better position
  if (path_fail) {
    const auto& pull_over_status =
        PlanningContext::Instance()->planning_status().pull_over();
    if (pull_over_status.has_position() &&
        pull_over_status.position().has_x() &&
        pull_over_status.position().has_y()) {
      const auto& reference_line = reference_line_info.reference_line();
      common::SLPoint pull_over_sl;
      reference_line.XYToSL(
          {pull_over_status.position().x(), pull_over_status.position().y()},
          &pull_over_sl);

      const double stop_line_s =
          pull_over_sl.s() -
          scenario_config_.s_distance_to_stop_for_open_space_parking();
      const std::string virtual_obstacle_id = "DEST_PULL_OVER_PREPARKING";
      const std::vector<std::string> wait_for_obstacle_ids;
      planning::util::BuildStopDecision(
          virtual_obstacle_id, stop_line_s, 1.0,
          StopReasonCode::STOP_REASON_PREPARKING, wait_for_obstacle_ids,
          "PULL-OVER-scenario", frame,
          &(frame->mutable_reference_line_info()->front()));

      ADEBUG << "Build a stop fence to pause ADC at a better position: id["
             << virtual_obstacle_id << "] s[" << stop_line_s << "]";

      const double adc_front_edge_s =
          reference_line_info.AdcSlBoundary().end_s();
      double distance = stop_line_s - adc_front_edge_s;
      constexpr double kPreparkingStopDistance = 1.0;
      constexpr double kPreparkingAngleDiff = 0.2;
      auto ref_point = reference_line.GetReferencePoint(adc_front_edge_s);
      double angle = common::math::AngleDiff(pull_over_status.theta(),
                                             ref_point.heading());
      if (distance <= kPreparkingStopDistance &&
          angle <= kPreparkingAngleDiff) {
        return FinishStage(false);
      }
    }
  }

  return StageStatus::RUNNING;
}

Stage::StageStatus PullOverStageApproach::FinishStage(const bool success) {
  if (success) {
    return FinishScenario();
  } else {
    next_stage_ = ScenarioConfig::PULL_OVER_RETRY_APPROACH_PARKING;
    return Stage::FINISHED;
  }
}

}  // namespace pull_over
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
