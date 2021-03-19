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

#include "cyber/component/component.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"

namespace apollo {
namespace perception {
namespace onboard {

class LidarOutputComponent : public cyber::Component<SensorFrameMessage> {
 public:
  LidarOutputComponent() = default;
  ~LidarOutputComponent() = default;

  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

 private:
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
};  // class LidarOutputComponent

CYBER_REGISTER_COMPONENT(LidarOutputComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
