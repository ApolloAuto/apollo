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

#include "modules/localization/msf/msf_localization.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using ::Eigen::Vector3d;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::ImuAdapter;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::time::Clock;

MSFLocalization::MSFLocalization()
    : monitor_(MonitorMessageItem::LOCALIZATION),
    map_offset_{FLAGS_map_offset_x, FLAGS_map_offset_y, FLAGS_map_offset_z} {}

Status MSFLocalization::Start() {
  AdapterManager::Init(FLAGS_msf_adapter_config_file);

  // start ROS timer, one-shot = false, auto-start = true
  const double duration = 1.0 / FLAGS_localization_publish_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &MSFLocalization::OnTimer, this);
  common::monitor::MonitorBuffer buffer(&monitor_);
  if (!AdapterManager::GetGps()) {
    buffer.ERROR() << "GPS input not initialized. Check file "
                   << FLAGS_msf_adapter_config_file;
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no GPS adapter");
  }
  AdapterManager::AddGpsCallback(&MSFLocalization::OnGps, this);
  if (!AdapterManager::GetImu()) {
    buffer.ERROR("IMU input not initialized. Check your adapter.conf file!");
    buffer.PrintLog();
    return Status(common::LOCALIZATION_ERROR, "no IMU adapter");
  }
  AdapterManager::AddImuCallback(&MSFLocalization::OnImu, this);
  CHECK(AdapterManager::GetPointCloud()) << "PointCloud is not initialized.";
  AdapterManager::AddPointCloudCallback(&MSFLocalization::OnPointCloud, this);
  return Status::OK();
}

Status MSFLocalization::Stop() {
  timer_.stop();
  return Status::OK();
}

void MSFLocalization::OnTimer(const ros::TimerEvent &event) {
}

void MSFLocalization::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  // set parameter: map_path

  // set parameter: lidar_extrinsic_file

  // set parameter: lidar_height_file

  // set parameter: debug_log_flag

  // set parameter: localization_mode

  // set parameter: local_utm_zone_id
}

void MSFLocalization::OnImu(const localization::Imu &imu_msg) {
}

void MSFLocalization::OnGps(const localization::Gps &gps_msg) {
}

void MSFLocalization::OnMeasure(
    const localization::IntegMeasure &measure_msg) {
}

void MSFLocalization::OnSinsPva(
    const localization::IntegSinsPva &sins_pva_msg) {
}

}  // namespace localization
}  // namespace apollo
