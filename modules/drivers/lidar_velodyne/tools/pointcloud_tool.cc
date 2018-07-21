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

#include <fstream>
#include <string>

#include "boost/filesystem.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/advertise_options.h"
#include "ros/ros.h"

#include "modules/common/log.h"
#include "modules/drivers/lidar_velodyne/common/util.h"
#include "modules/drivers/lidar_velodyne/pointcloud/velodyne_parser.h"
#include "modules/drivers/lidar_velodyne/tools/pointcloud_tool.h"
#include "modules/drivers/lidar_velodyne/tools/tools_gflags.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

using apollo::common::Status;
using apollo::common::ErrorCode;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::GetProtoFromFile;

std::string PointCloudTool::Name() const { return FLAGS_tools_module_name; }

Status PointCloudTool::Init() {
  Status error(ErrorCode::DRIVER_ERROR_VELODYNE);
  if (!GetProtoFromFile(FLAGS_tools_conf_file, &conf_)) {
    AERROR << "fail to load conf file.";
    return error;
  }

  AdapterManager::Init(FLAGS_tools_adapter_config_filename);
  if (FLAGS_open_pointcloud_dump) {
    RETURN_VAL_IF_NULL(AdapterManager::GetPointCloud(), error);
    AdapterManager::AddPointCloudCallback(&PointCloudTool::pointcloud_dump,
                                          this);
  }

  if (FLAGS_open_pointcloud_convert) {
    RETURN_VAL_IF_NULL(AdapterManager::GetPointCloudRaw0(), error);
    converter_ = new Converter();

    if (nullptr == converter_ ||
        !converter_->init(conf_.mutable_module_conf())) {
      AERROR << "new or init converter fail.";
      return error;
    }
    AdapterManager::AddVelodyneRaw0Callback(&PointCloudTool::pointcloud_convert,
                                            this);
  }

  if (FLAGS_open_pointcloud_compensate) {
    RETURN_VAL_IF_NULL(AdapterManager::GetPointCloud(), error);
    compensator_ = new Compensator(conf_.module_conf());
    RETURN_VAL_IF_NULL(compensator_, error);
    AdapterManager::AddPointCloudRaw0Callback(
        &PointCloudTool::pointcloud_compensate, this);
  }

  return Status::OK();
}

Status PointCloudTool::Start() { return Status::OK(); }

void PointCloudTool::Stop() {
  if (nullptr != converter_) {
    delete converter_;
    converter_ = nullptr;
  }

  if (nullptr != compensator_) {
    delete compensator_;
    compensator_ = nullptr;
  }
}

void PointCloudTool::pointcloud_dump(const sensor_msgs::PointCloud2& message) {
  VPointCloud msg;
  pcl::fromROSMsg(message, msg);
  std::string ordered_file_path =
      conf_.save_folder() + "/" + conf_.file_prefix() +
      boost::lexical_cast<std::string>(msg.header.seq) + ".msg";
  dump_msg<VPointCloud>(msg, ordered_file_path);
}

void PointCloudTool::pointcloud_convert(
    const velodyne_msgs::VelodyneScanUnified& message) {
  velodyne_msgs::VelodyneScanUnifiedPtr raw(
      new velodyne_msgs::VelodyneScanUnified());
  sensor_msgs::PointCloud2Ptr pointcloud(new sensor_msgs::PointCloud2());

  *raw = message;
  if (!converter_->convert_packets_to_pointcloud(raw, pointcloud)) {
    AERROR << "converter scan to pointcloud fail.";
    return;
  }
  AdapterManager::PublishPointCloudRaw0(*pointcloud);
}

void PointCloudTool::pointcloud_compensate(
    const sensor_msgs::PointCloud2& message) {
  sensor_msgs::PointCloud2Ptr pt(new sensor_msgs::PointCloud2());
  sensor_msgs::PointCloud2Ptr comper_pt(new sensor_msgs::PointCloud2());

  *pt = message;
  if (!compensator_->pointcloud_compensate(pt, comper_pt)) {
    AERROR << "pointcloud compensate fail.";
    return;
  }
  AdapterManager::PublishPointCloud(*comper_pt);
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
