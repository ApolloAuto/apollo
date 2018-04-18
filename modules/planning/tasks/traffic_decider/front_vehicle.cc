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

#include "modules/planning/tasks/traffic_decider/front_vehicle.h"

#include <string>

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::util::WithinBound;
using apollo::hdmap::PathOverlap;

CIPV::CIPV(const TrafficRuleConfig& config) : TrafficRule(config) {}

bool CIPV::ApplyRule(Frame* frame, ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  return true;
}

}  // namespace planning
}  // namespace apollo
