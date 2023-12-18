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

#include "modules/planning/scenarios/pull_over/stage_approach.h"

#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/scenarios/pull_over/context.h"
#include "modules/planning/scenarios/pull_over/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult PullOverStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  const ScenarioPullOverConfig& scenario_config =
      GetContextAs<PullOverContext>()->scenario_config;

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "PullOverStageApproach planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  PullOverState state =
      CheckADCPullOver(injector_->vehicle_state(), reference_line_info,
                       scenario_config, injector_->planning_context());

  if (state == PullOverState::PASS_DESTINATION ||
      state == PullOverState::PARK_COMPLETE) {
    return FinishStage(true);
  } else if (state == PullOverState::PARK_FAIL) {
    return FinishStage(false);
  }

  // check path_data to fail sooner
  bool path_fail = false;
  const auto& candidate_path_data = reference_line_info.GetCandidatePathData();
  if (!candidate_path_data.empty()) {
    for (const auto& path_data : candidate_path_data) {
      if (path_data.path_label().find("pullover") == std::string::npos) {
        break;
      }

      const double min_distance_to_end = FLAGS_path_bounds_decider_resolution *
                                         FLAGS_num_extra_tail_bound_point;
      for (size_t i = path_data.discretized_path().size(); i >= 1; --i) {
        if (path_data.frenet_frame_path().back().s() -
                path_data.frenet_frame_path()[i - 1].s() <
            min_distance_to_end) {
          continue;
        }
        // check the last adc_position planned
        const auto& path_point = path_data.discretized_path()[i];
        PullOverState state = CheckADCPullOverPathPoint(
            reference_line_info, scenario_config, path_point,
            injector_->planning_context());
        if (state == PullOverState::PARK_FAIL) {
          path_fail = true;
        }
        break;
      }
    }
  }

  // add a stop fence for adc to pause at a better position
  if (path_fail) {
    const auto& pull_over_status =
        injector_->planning_context()->planning_status().pull_over();
    if (pull_over_status.has_position() &&
        pull_over_status.position().has_x() &&
        pull_over_status.position().has_y()) {
      const auto& reference_line = reference_line_info.reference_line();
      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

      const double stop_line_s =
          pull_over_sl.s() -
          scenario_config.s_distance_to_stop_for_open_space_parking();
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
      static constexpr double kPreparkingStopDistance = 1.0;
      static constexpr double kPreparkingAngleDiff = 0.2;
      auto ref_point = reference_line.GetReferencePoint(adc_front_edge_s);
      double angle = common::math::AngleDiff(pull_over_status.theta(),
                                             ref_point.heading());
      if (distance <= kPreparkingStopDistance &&
          angle <= kPreparkingAngleDiff) {
        return FinishStage(false);
      }
    }
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult PullOverStageApproach::FinishStage(const bool success) {
  if (success) {
    return FinishScenario();
  } else {
    next_stage_ = "PULL_OVER_RETRY_APPROACH_PARKING";
    return StageResult(StageStatusType::FINISHED);
  }
}

}  // namespace planning
}  // namespace apollo
