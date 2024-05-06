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

#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/stage_creep.h"

#include <string>

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/speed_profile_generator.h"
#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

bool TrafficLightUnprotectedLeftTurnStageCreep::Init(
    const StagePipeline& config,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_dir, void* context) {
  CHECK_NOTNULL(context);
  bool ret = Stage::Init(config, injector, config_dir, context);
  if (!ret) {
    AERROR << Name() << "init failed!";
    return false;
  }
  return ret;
}

StageResult TrafficLightUnprotectedLeftTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  auto context = GetContextAs<TrafficLightUnprotectedLeftTurnContext>();
  const ScenarioTrafficLightUnprotectedLeftTurnConfig& scenario_config =
      context->scenario_config;

  if (!pipeline_config_.enabled()) {
    return FinishStage();
  }

  // Run creep decider.
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable";
      break;
    }

    const auto ret = ProcessCreep(frame, &reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run CreepDecider ], Error message: "
             << ret.error_message();
      break;
    }
  }

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageCreep planning error";
  }

  if (context->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string traffic_light_overlap_id =
      context->current_traffic_light_overlap_ids[0];
  PathOverlap* current_traffic_light_overlap =
      reference_line_info.GetOverlapOnReferenceLine(traffic_light_overlap_id,
                                                    ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);

  // creep
  // note: don't check traffic light color while creeping on right turn
  const double wait_time = Clock::NowInSeconds() - context->creep_start_time;
  const double timeout_sec = scenario_config.creep_timeout_sec();

  double creep_stop_s = GetCreepFinishS(current_traffic_light_overlap->end_s,
                                        *frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (CheckCreepDone(*frame, reference_line_info,
                     current_traffic_light_overlap->end_s, wait_time,
                     timeout_sec)) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

const CreepStageConfig&
TrafficLightUnprotectedLeftTurnStageCreep::GetCreepStageConfig() const {
  return GetContextAs<TrafficLightUnprotectedLeftTurnContext>()
      ->scenario_config.creep_stage_config();
}

StageResult TrafficLightUnprotectedLeftTurnStageCreep::FinishStage() {
  next_stage_ = "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE";
  return StageResult(StageStatusType::FINISHED);
}

}  // namespace planning
}  // namespace apollo
