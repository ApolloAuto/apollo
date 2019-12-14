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
 */

#pragma once

#include <mutex>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/third_party_perception/proto/radar_obstacle.pb.h"

#include "modules/common/status/status.h"

/**
 * @namespace apollo::third_party_perception
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {

class ThirdPartyPerception {
 public:
  std::string Name() const;
  apollo::common::Status Init();
  apollo::common::Status Start();
  void Stop();

  // Upon receiving mobileye data
  void OnMobileye(const apollo::drivers::Mobileye& message);
  // Upon receiving esr radar data
  void OnDelphiESR(const apollo::drivers::DelphiESR& message);
  // Upon receiving conti radar data
  void OnContiRadar(const apollo::drivers::ContiRadar& message);
  // Upon receiving localization data
  void OnLocalization(
      const apollo::localization::LocalizationEstimate& message);
  // Upon receiving chassis data
  void OnChassis(const apollo::canbus::Chassis& message);
  // publish perception obstacles when timer is triggered
  bool Process(apollo::perception::PerceptionObstacles* const response);

 private:
  std::mutex third_party_perception_mutex_;
  apollo::perception::PerceptionObstacles mobileye_obstacles_;
  apollo::perception::PerceptionObstacles radar_obstacles_;
  apollo::localization::LocalizationEstimate localization_;
  apollo::canbus::Chassis chassis_;
  RadarObstacles current_radar_obstacles_;
  RadarObstacles last_radar_obstacles_;
};

}  // namespace third_party_perception
}  // namespace apollo
