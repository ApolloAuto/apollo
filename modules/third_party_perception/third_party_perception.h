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

#ifndef MODEULES_THIRD_PARTY_PERCEPTION_THIRD_PARTY_PERCEPTION_H_
#define MODEULES_THIRD_PARTY_PERCEPTION_THIRD_PARTY_PERCEPTION_H_

#include <map>
#include <mutex>
#include <queue>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/third_party_perception/proto/radar_obstacle.pb.h"
#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::third_party_perception
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {

class ThirdPartyPerception : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  // Upon receiving mobileye data
  void OnMobileye(const apollo::drivers::Mobileye& message);
  // Upon receiving esr radar data
  void OnDelphiESR(const apollo::drivers::DelphiESR& message);
  // Upon receiving conti radar data
  void OnContiRadar(const apollo::drivers::ContiRadar& message);
  // Upon receiving localization data
  void OnLocalization(
      const apollo::localization::LocalizationEstimate& message);
  // Upont receiving chassis data
  void OnChassis(const apollo::canbus::Chassis& message);
  // publish perception obstacles when timer is triggered
  void OnTimer(const ros::TimerEvent&);

  ros::Timer timer_;
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

#endif  // MODULES_THIRD_PARTY_PERCEPTION_THIRD_PARTY_PERCEPTION_H_
