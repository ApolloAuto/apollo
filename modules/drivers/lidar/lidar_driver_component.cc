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
#include "modules/drivers/lidar/lidar_driver_component.h"

#include "modules/drivers/lidar/proto/lidar_parameter.pb.h"

namespace apollo {
namespace drivers {
namespace lidar {

LidarDriverComponent::LidarDriverComponent() {}
bool LidarDriverComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "load config error, file:" << config_file_path_;
    return false;
  }
  node_ = apollo::cyber::CreateNode("drivers_lidar");
  AINFO << "conf:" << conf_.DebugString();
  LidarDriverFactory::Instance()->RegisterLidarClients();
  driver_ = LidarDriverFactory::Instance()->CreateLidarDriver(node_, conf_);
  if (driver_ == nullptr || !driver_->Init()) {
    AERROR << "driver init error";
    return false;
  }
  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
