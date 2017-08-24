/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/traffic_decider/back_side_vehicles.h"

namespace apollo {
namespace planning {

BackSideVehicles::BackSideVehicles() : TrafficRule("BackSideVehicles") {}

bool BackSideVehicles::ApplyRule(ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& reference_line = reference_line_info->reference_line();
  const auto& adc_boundary = reference_line_info->ADCBoundary();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (path_obstacle->perception_sl_boundary().end_s() >=
        adc_boundary.end_s()) {
      continue;
    }

    const auto& bounding_box =
        path_obstacle->Obstacle()->PerceptionBoundingBox();
    if (reference_line.HasOverlap(bounding_box) ||
        reference_line_info->IsOnLeftLane(bounding_box.center()) ||
        reference_line_info->IsOnRightLane(bounding_box.center())) {
      ObjectDecisionType ignore;
      ignore.mutable_ignore();
      path_decision->AddLateralDecision(name(), path_obstacle->Id(), ignore);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
