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
#include "modules/common_msgs/sensor_msgs/mobileye.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"

/**
 * @namespace apollo::third_party_perception::conversion_mobileye
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace conversion_mobileye {

apollo::perception::PerceptionObstacles MobileyeToPerceptionObstacles(
    const apollo::drivers::Mobileye& mobileye,
    const apollo::localization::LocalizationEstimate& localization,
    const apollo::canbus::Chassis& chassis);

}  // namespace conversion_mobileye
}  // namespace third_party_perception
}  // namespace apollo
