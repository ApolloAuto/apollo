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

#include <string>
#include <vector>

#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/stage_intersection_cruise.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

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
using hdmap::PathOverlap;

Stage::StageStatus StageIntersectionCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StageIntersectionCruise plan error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // check if the traffic_light is still along referenceline
  std::string traffic_light_overlap_id = GetContext()->traffic_light_id;
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  if (traffic_light_overlap_it == traffic_light_overlaps.end()) {
    return FinishScenario();
  }

  // check pass intersection
  // TODO(all): update when pnc-junction is ready
  constexpr double kIntersectionLength = 10.0;  // unit: m
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  if (adc_back_edge_s - traffic_light_overlap_it->end_s > kIntersectionLength) {
    return FinishStage();
  }

  return Stage::RUNNING;
}

Stage::StageStatus StageIntersectionCruise::FinishStage() {
  return FinishScenario();
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
