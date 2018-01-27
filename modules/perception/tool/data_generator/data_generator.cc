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

#include "modules/perception/tool/data_generator/data_generator.h"

#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace perception {
namespace data_generator {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::perception::pcl_util::PointXYZIT;

std::string DataGenerator::Name() const {
  return "Perception Data Generator.";
}

Status DataGenerator::Init() {
  const std::string config_file =
      "/apollo/modules/perception/tool/data_generator/conf/adapter.conf";
  AdapterManager::Init(config_file);

  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  AdapterManager::AddPointCloudCallback(&DataGenerator::OnPointCloud, this);

  return Status::OK();
}

void DataGenerator::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  pcl::PointCloud<PointXYZIT> cld;
  pcl::fromROSMsg(message, cld);

  AINFO << "PointCloud size = " << cld.points.size();

  // localization
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();

  // chassis
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();

  Status status =
      VehicleStateProvider::instance()->Update(localization, chassis);
  VehicleState vehicle_state =
      VehicleStateProvider::instance()->vehicle_state();

  AINFO << "VehicleState updated.";

  // TODO(All): label point cloud and generate data automatically below
}

Status DataGenerator::Start() {
  return Status::OK();
}

void DataGenerator::Stop() {}

}  // namespace data_generator
}  // namespace perception
}  // namespace apollo
