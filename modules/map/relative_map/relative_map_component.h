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

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/map/relative_map/proto/relative_map_config.pb.h"

namespace apollo {
namespace relative_map {

class RelativeMapComponent final
    : public cybertron::Component<perception::PerceptionObstacles,
                                  canbus::Chassis,
                                  localization::LocalizationEstimate> {
 public:
  RelativeMapComponent() = default;
  ~RelativeMapComponent() = default;
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<perception::PerceptionObstacles>&
            prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
            localization_estimate) override;
 private:
  std::shared_ptr<::apollo::cybertron::Writer<MapMsg>>
      relative_map_writer_ = nullptr;

};

CYBERTRON_REGISTER_COMPONENT(RelativeMapComponent)

}  // namespace relative_map
}  // namespace apollo

#endif  // MODULES_MAP_RELATIVE_MAP_RELATIVE_MAP_COMPONENT_H_

