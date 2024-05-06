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

#include "modules/planning/scenarios/park_and_go/proto/park_and_go.pb.h"
#include "modules/common/math/vec2d.h"

namespace apollo {
namespace common {
class VehicleStateProvider;
}
}  // namespace apollo

namespace apollo {
namespace planning {
class Frame;
class ReferenceLineInfo;
}  // namespace planning
}  // namespace apollo

namespace apollo {
namespace planning {

bool CheckADCSurroundObstacles(const common::math::Vec2d adc_position,
                               const double adc_heading, Frame* frame,
                               const double front_obstacle_buffer);

bool CheckADCHeading(const common::math::Vec2d adc_position,
                     const double adc_heading,
                     const ReferenceLineInfo& reference_line_info,
                     const double heading_buffer);

bool CheckADCReadyToCruise(
    const common::VehicleStateProvider* vehicle_state_provider, Frame* frame,
    const apollo::planning::ScenarioParkAndGoConfig& scenario_config);

}  // namespace planning
}  // namespace apollo
