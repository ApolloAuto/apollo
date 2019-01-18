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

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/tasks/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::PathOverlap;
using perception::TrafficLight;

Stage::StageStatus StageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "TrafficLightProtectedStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // check if the traffic_light is still along reference_line
  std::string traffic_light_id = GetContext()->traffic_light_id;
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_id;
                   });
  if (traffic_light_overlap_it == traffic_light_overlaps.end()) {
    return FinishScenario();
  }

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_traffic_light =
      adc_front_edge_s - traffic_light_overlap_it->start_s;
  constexpr double kPassStopLineBuffer = 1.0;  // unit: m

  // passed stop line too far
  if (distance_adc_pass_traffic_light > kPassStopLineBuffer) {
    return FinishStage();
  }

  // check on traffic light color
  if (PlanningContext::GetScenarioInfo()->traffic_light_color ==
      TrafficLight::GREEN) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus StageStop::FinishScenario() {
  PlanningContext::GetScenarioInfo()->stop_done_overlap_id = "";

  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

Stage::StageStatus StageStop::FinishStage() {
  PlanningContext::GetScenarioInfo()->stop_done_overlap_id =
      GetContext()->traffic_light_id;
  next_stage_ = ScenarioConfig::TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE;
  return Stage::FINISHED;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
