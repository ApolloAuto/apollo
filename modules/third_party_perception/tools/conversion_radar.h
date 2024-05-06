/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 */

#pragma once

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/sensor_msgs/conti_radar.pb.h"
#include "modules/common_msgs/sensor_msgs/delphi_esr.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/third_party_perception/proto/radar_obstacle.pb.h"

/**
 * @namespace apollo::third_party_perception::conversion_radar
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace conversion_radar {

RadarObstacles DelphiToRadarObstacles(
    const apollo::drivers::DelphiESR& delphi_esr,
    const apollo::localization::LocalizationEstimate& localization,
    const RadarObstacles& last_radar_obstacles);

RadarObstacles ContiToRadarObstacles(
    const apollo::drivers::ContiRadar& conti_radar,
    const apollo::localization::LocalizationEstimate& localization,
    const RadarObstacles& last_radar_obstacles,
    const apollo::canbus::Chassis& chassis);

apollo::perception::PerceptionObstacles RadarObstaclesToPerceptionObstacles(
    const RadarObstacles& radar_obstacles);

}  // namespace conversion_radar
}  // namespace third_party_perception
}  // namespace apollo
