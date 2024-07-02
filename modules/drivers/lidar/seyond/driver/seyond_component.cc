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
#include "modules/drivers/lidar/seyond/driver/seyond_component.h"
#include "modules/drivers/lidar/seyond/driver/seyond_driver.h"


namespace apollo {
namespace drivers {
namespace seyond {

bool SeyondComponent::Init() {
  if (!GetProtoConfig(&seyond_conf_)) {
    AERROR << " load config error, file:";
    return false;
  }
  AINFO << " conf:\n" << seyond_conf_.DebugString();

  driver_.reset(new SeyondDriver(node_, seyond_conf_));
  driver_->init();
  driver_->start();
  return true;
}

}  // namespace seyond
}  // namespace drivers
}  // namespace apollo
