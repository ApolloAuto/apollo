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

#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/stage_creep.h"

#include <string>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/creep_decider/creep_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::PathOverlap;

Stage::StageStatus TrafficLightUnprotectedRightTurnStageCreep::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Creep";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  if (!config_.enabled()) {
    return FinishStage();
  }

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightUnprotectedRightTurnStageCreep planning error";
  }

  if (GetContext()->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  const std::string traffic_light_overlap_id =
      GetContext()->current_traffic_light_overlap_ids[0];
  PathOverlap* current_traffic_light_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                traffic_light_overlap_id,
                                                ReferenceLineInfo::SIGNAL);
  if (!current_traffic_light_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  reference_line_info.SetJunctionRightOfWay(
      current_traffic_light_overlap->start_s, false);

  // creep
  // note: don't check traffic light color while creeping on right turn
  const double wait_time =
      Clock::NowInSeconds() - GetContext()->creep_start_time;
  const double timeout_sec = scenario_config_.creep_timeout_sec();
  auto* task = dynamic_cast<CreepDecider*>(FindTask(TaskConfig::CREEP_DECIDER));
  if (task == nullptr) {
    AERROR << "task is nullptr";
    return FinishStage();
  }

  double creep_stop_s = current_traffic_light_overlap->end_s +
                        task->FindCreepDistance(*frame, reference_line_info);
  const double distance =
      creep_stop_s - reference_line_info.AdcSlBoundary().end_s();
  if (distance <= 0.0) {
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(0.0, 0);
  }

  if (task->CheckCreepDone(*frame, reference_line_info,
                           current_traffic_light_overlap->end_s, wait_time,
                           timeout_sec)) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightUnprotectedRightTurnStageCreep::FinishStage() {
  next_stage_ =
      StageType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
