/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <string>

#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace util {

enum PullOverStatus {
  UNKNOWN = 0,
  APPOACHING = 1,
  PARK_COMPLETE = 2,
  PARK_FAIL = 3,
  PASS_DESTINATION = 4,
};

hdmap::PathOverlap* GetOverlapOnReferenceLine(
    const ReferenceLineInfo& reference_line_info, const std::string& overlap_id,
    const ReferenceLineInfo::OverlapType& overlap_type);

PullOverStatus CheckADCPullOver(const ReferenceLineInfo& reference_line_info,
                                const ScenarioPullOverConfig& scenario_config);

PullOverStatus CheckADCPullOverPathPoint(
    const ReferenceLineInfo& reference_line_info,
    const ScenarioPullOverConfig& scenario_config,
    const common::PathPoint& path_point);

PullOverStatus CheckADCPullOverOpenSpace(
    const ScenarioPullOverConfig& scenario_config);

bool CheckPullOverPositionBySL(const ReferenceLineInfo& reference_line_info,
                               const ScenarioPullOverConfig& scenario_config,
                               const common::math::Vec2d& adc_position,
                               const double adc_theta,
                               const common::math::Vec2d& target_position,
                               const double target_theta, const bool check_s);

bool CheckPullOverPositionByDistance(
    const ScenarioPullOverConfig& scenario_config,
    const common::math::Vec2d& adc_position, const double adc_theta,
    const common::math::Vec2d& target_position, const double target_theta);

}  // namespace util
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
