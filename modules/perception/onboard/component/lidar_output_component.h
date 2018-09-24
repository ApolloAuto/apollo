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
#ifndef PERCEPTION_ONBOARD_COMPONENT_LIDAR_OUTPUT_COMPONENT_H_
#define PERCEPTION_ONBOARD_COMPONENT_LIDAR_OUTPUT_COMPONENT_H_

#include "cybertron/component/component.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"

namespace apollo {
namespace perception {
namespace onboard {

class LidarOutputComponent : public cybertron::Component<SensorFrameMessage> {
 public:
  LidarOutputComponent() = default;
  ~LidarOutputComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

 private:
  std::shared_ptr<apollo::cybertron::Writer<PerceptionObstacles>> writer_;

};  // class LidarOutputComponent

CYBERTRON_REGISTER_COMPONENT(LidarOutputComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_ONBOARD_COMPONENT_LIDAR_OUTPUT_COMPONENT_H_
