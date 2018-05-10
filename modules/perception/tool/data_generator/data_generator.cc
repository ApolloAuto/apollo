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
#include "modules/perception/tool/data_generator/velodyne64.h"

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

std::string DataGenerator::Name() const { return "data_generator"; }

Status DataGenerator::Init() {
  RegisterSensors();
  AdapterManager::Init(FLAGS_data_generator_adapter_config_filename);

  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_data_generator_config_file,
                                               &data_generator_info_))
      << "failed to load data generator config file "
      << FLAGS_data_generator_config_file;
  sensor_configs_ = data_generator_info_.config();

  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  const std::string file_name = FLAGS_data_file_prefix + FLAGS_data_file_name +
                                "_" + std::to_string(num_data_frame_);
  data_file_ = new std::ofstream(file_name);
  CHECK(data_file_->is_open()) << file_name;

  return Status::OK();
}

void DataGenerator::RegisterSensors() {
  sensor_factory_.Register(SensorConfig::VELODYNE64,
                           [](const SensorConfig& config) -> Sensor* {
                             return new Velodyne64(config);
                           });
}

void DataGenerator::OnTimer(const ros::TimerEvent&) { RunOnce(); }

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
  Process();

  std::string str;
  for (auto& obs : data_generator_info_.obstacle()) {
    str += obs.id() + "|, ";
    for (int i = 0; i < obs.polygon_point_size(); ++i) {
      str += std::to_string(obs.polygon_point(i).x()) + ", " +
             std::to_string(obs.polygon_point(i).y());
      if (i + 1 != obs.polygon_point_size()) {
        str += "| ";
      }
    }
  }
  (*data_file_) << str << "; ";
  for (auto& sensor : sensors_) {
    (*data_file_) << sensor->data() << "; ";
  }
}

bool DataGenerator::Process() {
  for (const auto& sensor_config : sensor_configs_) {
    auto sensor =
        sensor_factory_.CreateObject(sensor_config.id(), sensor_config);
    if (!sensor) {
      AERROR << "Could not find sensor " << sensor_config.DebugString();
      continue;
    }
    sensor->Process();
    ADEBUG << "Processed sensor "
           << SensorConfig::SensorId_Name(sensor_config.id());
  }
  return true;
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
