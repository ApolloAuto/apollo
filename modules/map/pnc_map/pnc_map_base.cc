/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file planning_map_base.cc
 */

#include "modules/map/pnc_map/pnc_map_base.h"

#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

DEFINE_double(
    look_backward_distance, 50,
    "look backward this distance when creating reference line from routing");

DEFINE_double(look_forward_short_distance, 180,
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(
    look_forward_long_distance, 250,
    "look forward this distance when creating reference line from routing");

bool PncMapBase::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {
  if (!IsValid(command)) {
    AERROR << "Input command is not valid!" << command.DebugString();
    return false;
  }
  last_command_ = command;
  return true;
}

double PncMapBase::LookForwardDistance(const double velocity) {
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  return forward_distance > FLAGS_look_forward_short_distance
             ? FLAGS_look_forward_long_distance
             : FLAGS_look_forward_short_distance;
}

bool PncMapBase::IsNewPlanningCommand(
    const planning::PlanningCommand &command) const {
  return IsNewPlanningCommand(last_command_, command);
}

bool PncMapBase::IsNewPlanningCommand(
    const planning::PlanningCommand &prev_command,
    const planning::PlanningCommand &new_command) {
  if (!new_command.is_motion_command()) {
    return false;
  }
  return !common::util::IsProtoEqual(prev_command, new_command);
}

}  // namespace planning
}  // namespace apollo
