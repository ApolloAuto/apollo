/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "cyber/cyber.h"

#include "modules/drivers/lidar/seyond/driver/seyond_driver.h"
#include "modules/drivers/lidar/seyond/proto/seyond_config.pb.h"



namespace apollo {
namespace drivers {
namespace seyond {

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class SeyondComponent : public ::apollo::cyber::Component<> {
 public:
  ~SeyondComponent() {
  }
  bool Init() override;

 private:
  std::shared_ptr<SeyondDriver> driver_{nullptr};
  apollo::drivers::seyond::Config seyond_conf_;
};

CYBER_REGISTER_COMPONENT(SeyondComponent)

}  // namespace seyond
}  // namespace drivers
}  // namespace apollo
