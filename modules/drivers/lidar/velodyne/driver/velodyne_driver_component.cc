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

VelodyneDriverComponent::~VelodyneDriverComponent() {
  if (device_thread_ && device_thread_->joinable()) {
    device_thread_->join();
  }
}

bool VelodyneDriverComponent::Init() {
  AINFO << "Velodyne driver component init";
  Config velodyne_config;
  if (!GetProtoConfig(&velodyne_config)) {
    return false;
  }
  AINFO << "Velodyne config: " << velodyne_config.DebugString();
  // start the driver
  writer_ = node_->CreateWriter<VelodyneScan>(velodyne_config.scan_channel());
  VelodyneDriver *driver = VelodyneDriverFactory::CreateDriver(velodyne_config);
  if (driver == nullptr) {
    return false;
  }
  driver_.reset(driver);
  driver_->Init();
  // spawn device poll thread
  device_thread_ = std::make_shared<std::thread>(
      std::bind(&VelodyneDriverComponent::device_poll, this));

  return true;
}

/** @brief Device poll thread main loop. */
void VelodyneDriverComponent::device_poll() {
  while (!apollo::cyber::IsShutdown()) {
    // poll device until end of file
    auto scan = std::make_shared<VelodyneScan>();
    bool ret = driver_->Poll(scan);
    if (ret) {
      common::util::FillHeader("velodyne", scan.get());
      writer_->Write(scan);
    } else {
      AWARN << "device poll failed";
    }
  }

  AERROR << "CompVelodyneDriver thread exit";
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
