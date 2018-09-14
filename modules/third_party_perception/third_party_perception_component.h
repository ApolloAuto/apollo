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

#ifndef MODULES_THIRD_PARTY_PERCEPTION_THIRD_PARTY_PERCEPTION_COMPONENT_H_
#define MODULES_THIRD_PARTY_PERCEPTION_THIRD_PARTY_PERCEPTION_COMPONENT_H_

#include <memory>

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"
#include "cybertron/component/timer_component.h"

#include "modules/third_party_perception/third_party_perception.h"

namespace apollo {
namespace third_party_perception {

class ThirdPartyPerceptionComponent final
    : public ::apollo::cybertron::TimerComponent {
 public:
  ThirdPartyPerceptionComponent() = default;
  ~ThirdPartyPerceptionComponent() = default;
 public:
  bool Init() override;
  bool Proc() override;
 private:
  std::shared_ptr<::apollo::cybertron::Reader
      <::apollo::drivers::Mobileye>> mobileye_reader_ = nullptr;
  std::shared_ptr<::apollo::cybertron::Reader
      <::apollo::drivers::DelphiESR>> delphi_esr_reader_ = nullptr;
  std::shared_ptr<::apollo::cybertron::Reader
      <::apollo::drivers::ContiRadar>> conti_radar_reader_ = nullptr;
  std::shared_ptr<::apollo::cybertron::Reader
      <::apollo::localization::LocalizationEstimate>> localization_reader_ = nullptr;
  std::shared_ptr
      <::apollo::cybertron::Reader<::apollo::canbus::Chassis>> chassis_reader_ = nullptr;
  std::shared_ptr<::apollo::cybertron::Writer
      <::apollo::perception::PerceptionObstacles>> writer_ = nullptr;
  ThirdPartyPerception perception_;
};

CYBERTRON_REGISTER_COMPONENT(ThirdPartyPerceptionComponent)
}  // namespace third_party_perception
}  // namespace apollo

#endif  // MODULES_THIRD_PARTY_PERCEPTION_THIRD_PARTY_PERCEPTION_COMPONENT_H_

