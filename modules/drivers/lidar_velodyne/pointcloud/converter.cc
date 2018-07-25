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

#include "modules/drivers/lidar_velodyne/pointcloud/converter.h"

#include "pcl/common/time.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/advertise_options.h"

#include "modules/drivers/lidar_velodyne/proto/velodyne_conf.pb.h"

#include "modules/common/log.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

Converter::Converter() : parser_(nullptr) {}
Converter::~Converter() {
  if (parser_ != nullptr) {
    delete parser_;
  }
}

bool Converter::ready() { return (parser_ != nullptr && parser_->ready()); }

bool Converter::pack(sensor_msgs::PointCloud2Ptr pointcloud) {
  if (parser_ == nullptr) {
    AERROR << "parser_ is null";
    return false;
  }

  if (!ready()) {
    AERROR << "not ready, can not pack.";
    return false;
  }

  VPointCloud::Ptr pcl_pointcloud = parser_->pack();
  if (!pcl_pointcloud || pcl_pointcloud->empty()) {
    AERROR << "generate vpointcloud fail.";
    return false;
  }

  if (config_.organized()) {
    parser_->order(pcl_pointcloud);
  }
  pcl::toROSMsg(*(pcl_pointcloud.get()), *(pointcloud.get()));

  return true;
}

bool Converter::append(const velodyne_msgs::VelodyneScanUnifiedPtr scan_msg) {
  return parser_->append(scan_msg);
}

bool Converter::init(const VelodyneConf& conf) {
  config_ = conf;
  parser_ = VelodyneParserFactory::create_parser(&config_);
  if (parser_ == nullptr) {
    AERROR << "velodyne create parser fail.";
    return false;
  }
  if (!parser_->setup()) {
    AERROR << " parser setup fail. ";
    return false;
  }
  return true;
}

bool Converter::convert_packets_to_pointcloud(
    const velodyne_msgs::VelodyneScanUnifiedPtr scan_msg,
    sensor_msgs::PointCloud2Ptr pointcloud) {
  if (!scan_msg || !pointcloud) {
    AERROR << "input data scan or pointcloud error";
    return false;
  }
  ADEBUG << "seq num: " << scan_msg->header.seq;

  VPointCloud::Ptr pcl_pointcloud(new VPointCloud());
  if (!pcl_pointcloud) {
    AERROR << "new VPointCloud fail.";
    return false;
  }

  parser_->generate_pointcloud(scan_msg, pcl_pointcloud);
  if (pcl_pointcloud->empty()) {
    AERROR << "generate vpointcloud fail.";
    return false;
  }

  if (config_.organized()) {
    ADEBUG << "reorder point cloud";
    parser_->order(pcl_pointcloud);
  }
  pcl::toROSMsg(*(pcl_pointcloud.get()), *(pointcloud.get()));

  return true;
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
