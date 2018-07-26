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

#include <string>

#include "modules/drivers/lidar_velodyne/tools/pcd_exporter.h"
#include "modules/drivers/lidar_velodyne/tools/pcd_exporter_app.h"
#include "modules/drivers/lidar_velodyne/tools/tools_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/apollo_app.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::GetProtoFromFile;

std::string PcdApp::Name() const {
  return FLAGS_tools_module_name;
}

Status PcdApp::Init() {
  VelodyneToolsConf conf;
  if (!GetProtoFromFile(FLAGS_tools_conf_file, &conf)) {
    AERROR << "fail to load conf file.";
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE);
  }

  AdapterManager::Init(FLAGS_tools_adapter_config_filename);
  if (nullptr == AdapterManager::GetPointCloud()) {
    AERROR << "init pointcloud adapter fail.";
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE);
  }

  exporter_ = new PCDExporter();
  if (nullptr == exporter_ || !exporter_->init(conf)) {
    AERROR << "new pcd exporter fail.";
    return Status(ErrorCode::DRIVER_ERROR_VELODYNE);
  }
  AdapterManager::AddPointCloudCallback(&PcdApp::OnPointCloud, this);

  return Status::OK();
}

Status PcdApp::Start() {
  AINFO << "pcd app start.";
  return Status::OK();
}

void PcdApp::Stop() {
  if (nullptr != exporter_) {
    delete exporter_;
    exporter_ = nullptr;
  }
}

void PcdApp::OnPointCloud(const sensor_msgs::PointCloud2& message) {
    exporter_->pcd_writer_callback(message);
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

