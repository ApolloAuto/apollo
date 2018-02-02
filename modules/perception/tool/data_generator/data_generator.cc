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

#include "modules/perception/tool/data_generator/data_generator.h"

#include "eigen_conversions/eigen_msg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/perception/tool/data_generator/common/data_generator_gflags.h"

namespace apollo {
namespace perception {
namespace data_generator {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::perception::pcl_util::PointCloud;
using apollo::perception::pcl_util::PointCloudPtr;
using apollo::perception::pcl_util::PointD;
using apollo::perception::pcl_util::PointXYZIT;
using Eigen::Affine3d;
using Eigen::Matrix4d;

std::string DataGenerator::Name() const {
  return "data_generator";
}

Status DataGenerator::Init() {
  AdapterManager::Init(FLAGS_data_generator_adapter_config_filename);

  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  const std::string file_name = FLAGS_data_file_prefix + FLAGS_data_file_name +
                                "_" + std::to_string(num_data_frame_);
  data_file_ = new std::ofstream(file_name);
  CHECK(data_file_->is_open()) << file_name;

  return Status::OK();
}

void DataGenerator::OnTimer(const ros::TimerEvent&) {
  RunOnce();
}

void DataGenerator::RunOnce() {
  AdapterManager::Observe();

  // point cloud
  if (AdapterManager::GetPointCloud()->Empty()) {
    AERROR << "PointCloud is NOT ready.";
    return;
  }
  // localization
  if (AdapterManager::GetLocalization()->Empty()) {
    AERROR << "Localization is NOT ready.";
    return;
  }
  // chassis
  if (AdapterManager::GetChassis()->Empty()) {
    AERROR << "Chassis is NOT ready.";
    return;
  }

  // Update VehicleState
  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Localization: " << localization.DebugString();
  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Chassis: " << chassis.DebugString();
  VehicleStateProvider::instance()->Update(localization, chassis);
  AINFO << "VehicleState updated.";

  // TODO(Liangliang): register more sensors and use factory to manager.
}

Status DataGenerator::Start() {
  constexpr double kDataGeneratorCycleDuration = 0.1;  // in seconds
  timer_ =
      AdapterManager::CreateTimer(ros::Duration(kDataGeneratorCycleDuration),
                                  &DataGenerator::OnTimer, this);
  AINFO << "DataGenerator started";
  return Status::OK();
}

void DataGenerator::Stop() {
  data_file_->close();
  delete data_file_;
}

}  // namespace data_generator
}  // namespace perception
}  // namespace apollo
