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

#include "modules/planning/scenarios/stop_sign/stop_sign_unprotected/stop_sign_unprotected_intersection_cruise.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/toolkits/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using apollo::hdmap::PathOverlap;
using common::TrajectoryPoint;

Stage::StageStatus StopSignUnprotectedIntersectionCruise::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: IntersectionCruise";
  CHECK_NOTNULL(frame);

  if (CheckPassIntersection(frame)) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }

  bool plan_ok = PlanningOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedIntersectionCruise plan error";
  }
  return Stage::RUNNING;
}

bool StopSignUnprotectedIntersectionCruise::CheckPassIntersection(
    Frame* frame) {
  // TODO(all): update when pnc-junction is ready
  constexpr double kIntersectionLength = 10.0;  // unit: m

  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  double stop_sign_oberlap_end_s = 0;
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  for (const PathOverlap& stop_sign_overlap : stop_sign_overlaps) {
    if (GetContext()->stop_sign_id == stop_sign_overlap.object_id) {
      stop_sign_oberlap_end_s = stop_sign_overlap.end_s;
      break;
    }
  }

  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  ADEBUG << "adc_back_edge_s[" << adc_back_edge_s
      << "] stop_sign_overlap_end_s[" << stop_sign_oberlap_end_s << "]";
  if (adc_back_edge_s - stop_sign_oberlap_end_s >
      kIntersectionLength) {
    return true;
  }
  return false;
}


}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
