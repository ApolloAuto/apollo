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

#include <vector>

#include "modules/planning/scenarios/traffic_light/protected/stage_stop.h"

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::TrajectoryPoint;
using perception::TrafficLight;

Stage::StageStatus TrafficLightProtectedStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightProtectedStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  bool traffic_light_all_done = true;
  for (const auto& traffic_light_overlap :
       PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps) {
    // check if the traffic_light is still along reference_line
    if (scenario::CheckTrafficLightDone(reference_line_info,
                                        traffic_light_overlap.object_id)) {
      continue;
    }

    // check on traffic light color
    auto signal_color =
        scenario::GetSignal(traffic_light_overlap.object_id).color();
    ADEBUG << "traffic_light_overlap_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s
           << "] color[" << signal_color << "]";
    if (signal_color != TrafficLight::GREEN) {
      traffic_light_all_done = false;
      break;
    }
  }

  if (traffic_light_all_done) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus TrafficLightProtectedStageStop::FinishScenario() {
  PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.clear();

  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

Stage::StageStatus TrafficLightProtectedStageStop::FinishStage() {
  PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.clear();
  for (const auto& traffic_light_overlap :
      PlanningContext::GetScenarioInfo()->current_traffic_light_overlaps) {
    PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.push_back(
        traffic_light_overlap.object_id);
  }

  next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
