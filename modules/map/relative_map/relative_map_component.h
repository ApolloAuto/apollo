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

#ifndef MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_COMPONENT_H_
#define MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_COMPONENT_H_

#include <memory>

#include "cybertron/component/timer_component.h"
#include "cybertron/cybertron.h"

#include "modules/map/relative_map/relative_map.h"

namespace apollo {
namespace relative_map {

class RelativeMapComponent final
    : public ::apollo::cybertron::TimerComponent {

 public:
  bool Init() override;
  bool Proc() override;

 private:
  bool InitReaders();

  std::shared_ptr<::apollo::cybertron::Writer<MapMsg>>
      relative_map_writer_ = nullptr;
  std::shared_ptr<cybertron::Reader<perception::PerceptionObstacles>>
      perception_reader_ = nullptr;
  std::shared_ptr<cybertron::Reader<canbus::Chassis>>
      chassis_reader_ = nullptr;
  std::shared_ptr<cybertron::Reader<localization::LocalizationEstimate>>
      localization_reader_ = nullptr;
  std::shared_ptr<cybertron::Reader<NavigationInfo>>
      navigation_reader_ = nullptr;

  RelativeMap relative_map_;
};

CYBERTRON_REGISTER_COMPONENT(RelativeMapComponent)
}  // namespace relative_map
}  // namespace apollo

#endif  // MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_COMPONENT_H_

