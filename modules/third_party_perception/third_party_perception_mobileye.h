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
#include <memory>

#include "modules/third_party_perception/third_party_perception_base.h"

#include "modules/common_msgs/sensor_msgs/conti_radar.pb.h"
#include "modules/common_msgs/sensor_msgs/delphi_esr.pb.h"
#include "modules/common_msgs/sensor_msgs/mobileye.pb.h"

/**
 * @namespace apollo::third_party_perception
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {

class ThirdPartyPerceptionMobileye : public ThirdPartyPerception {
 public:
  explicit ThirdPartyPerceptionMobileye(apollo::cyber::Node* const node);
  ThirdPartyPerceptionMobileye() = default;
  ~ThirdPartyPerceptionMobileye() = default;
  // Upon receiving mobileye data
  void OnMobileye(const apollo::drivers::Mobileye& message);
  // Upon receiving conti radar data
  void OnContiRadar(const apollo::drivers::ContiRadar& message);
  // Upon receiving esr radar data
  void OnDelphiESR(const apollo::drivers::DelphiESR& message);

  bool Process(
      apollo::perception::PerceptionObstacles* const response) override;

 private:
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::Mobileye>>
      mobileye_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::DelphiESR>>
      delphi_esr_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::ContiRadar>>
      conti_radar_reader_ = nullptr;
  apollo::perception::PerceptionObstacles radar_obstacles_;
  apollo::perception::PerceptionObstacles eye_obstacles_;
};

}  // namespace third_party_perception
}  // namespace apollo
