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

#pragma once

#include <memory>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/relative_map/relative_map.h"

namespace apollo {
namespace relative_map {

class RelativeMapComponent final : public ::apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 private:
  bool InitReaders();

  std::shared_ptr<::apollo::cyber::Writer<MapMsg>> relative_map_writer_ =
      nullptr;
  std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>>
      perception_reader_ = nullptr;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_ = nullptr;
  std::shared_ptr<cyber::Reader<NavigationInfo>> navigation_reader_ = nullptr;

  std::shared_ptr<common::VehicleStateProvider> vehicle_state_provider_;
  RelativeMap relative_map_;
};

CYBER_REGISTER_COMPONENT(RelativeMapComponent)
}  // namespace relative_map
}  // namespace apollo
