/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/velodyne/driver/velodyne_driver_component.h"

#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace drivers {
namespace velodyne {

bool VelodyneDriverComponent::Init() {
  AINFO << "Velodyne driver component init";
  Config velodyne_config;
  if (!GetProtoConfig(&velodyne_config)) {
    return false;
  }
  AINFO << "Velodyne config: " << velodyne_config.DebugString();
  // start the driver
  std::shared_ptr<::apollo::cyber::Node> node =
      apollo::cyber::CreateNode("lidar_drivers");
  VelodyneDriver *driver =
      VelodyneDriverFactory::CreateDriver(node, velodyne_config);
  if (driver == nullptr) {
    return false;
  }
  dvr_.reset(driver);
  dvr_->Init();
  // spawn device poll thread
  runing_ = true;
  return true;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
