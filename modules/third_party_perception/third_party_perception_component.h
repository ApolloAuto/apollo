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
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/third_party_perception/third_party_perception_base.h"
#include "modules/third_party_perception/third_party_perception_smartereye.h"
#include "modules/third_party_perception/third_party_perception_mobileye.h"

namespace apollo {
namespace third_party_perception {

class ThirdPartyPerceptionComponent final
    : public apollo::cyber::TimerComponent {
 public:
  ThirdPartyPerceptionComponent() = default;
  ~ThirdPartyPerceptionComponent() = default;

 public:
  bool Init() override;
  bool Proc() override;

 private:
  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::PerceptionObstacles>>
      writer_ = nullptr;
  std::shared_ptr<ThirdPartyPerception> perception_ = nullptr;
};

CYBER_REGISTER_COMPONENT(ThirdPartyPerceptionComponent)
}  // namespace third_party_perception
}  // namespace apollo
