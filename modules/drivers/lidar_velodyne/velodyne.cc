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

#include "modules/drivers/lidar_velodyne/velodyne.h"

#include <memory>

#include "ros/include/std_msgs/String.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/drivers/lidar_velodyne/common/velodyne_gflags.h"
#include "modules/drivers/lidar_velodyne/driver/driver.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

using apollo::common::adapter::AdapterManager;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::util::GetProtoFromFile;
using velodyne_msgs::VelodyneScanUnifiedPtr;

std::string Velodyne::Name() const { return FLAGS_velodyne_module_name; }

Status Velodyne::Init() {
  AINFO << "Velodyne init, starting ...";
  return Status::OK();
}

Status Velodyne::Start() {
  std::shared_ptr<std::thread> driver_thread(
      new std::thread(std::bind(&DriverNodelet::OnInit, &driver_nodelet_)));
  threads_.push_back(driver_thread);

  std::shared_ptr<std::thread> cloud_thread(
      new std::thread(std::bind(&CloudNodelet::OnInit, &cloud_nodelet_)));
  threads_.push_back(cloud_thread);

  std::shared_ptr<std::thread> transform_thread(new std::thread(
      std::bind(&TransformNodelet::OnInit, &transform_nodelet_)));
  threads_.push_back(transform_thread);

  std::shared_ptr<std::thread> ringcolors_thread(new std::thread(
      std::bind(&RingColorsNodelet::OnInit, &ringcolors_nodelet_)));
  threads_.push_back(ringcolors_thread);

  ADEBUG << "Velodyne start done!";
  return Status::OK();
}

void Velodyne::Stop() {
  AINFO << "Velodyne Stopping ...";
  running_ = false;

  for (size_t i = 0; i < threads_.size(); ++i) {
    if (threads_[i]->joinable()) {
      threads_[i]->join();
    }
  }
  threads_.clear();
  AINFO << "Velodyne Stopped.";
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
