/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
 * @file driving_state_generator.h
 */

#pragma once

#include <list>

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class DrivingStateGenerator {
 public:
  void generate(const ReferenceLineInfo* reference_line_info,
                const EgoInfo* ego_info,
                planning_internal::ADCDrivingState* const state);
  void add_path_info(const ReferenceLineInfo* reference_line_info,
                     planning_internal::ADCDrivingState* const state);
  void add_lane_info(const ReferenceLineInfo* reference_line_info,
                     planning_internal::ADCDrivingState* const state);
  void add_nudge_info(const ReferenceLineInfo* reference_line_info,
                     planning_internal::ADCDrivingState* const state);
};

}  // namespace planning
}  // namespace apollo