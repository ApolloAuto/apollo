/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/drivers/lidar/robosense/parser/convert.h"

#include <memory>

#include <pcl/common/time.h>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace robosense {

/** @brief Constructor. */
Convert::Convert(const apollo::drivers::suteng::SutengConfig& robo_config) {
  config_ = robo_config;
  config_.set_view_direction(0.0);
  config_.set_view_width(2.0 * M_PI);
}

bool Convert::Init() {
  parser_ = RobosenseParserFactory::create_parser(config_);
  if (parser_ == nullptr) {
    AERROR << " can not create velodyen parser";
    return false;
  }
  parser_->setup();
  return true;
}

Convert::~Convert() {
  if (parser_ != nullptr) {
    delete parser_;
  }
}

uint32_t Convert::GetPointSize() { return parser_->GetPointSize(); }

/** @brief Callback for raw scan messages. */
void Convert::convert_robosense_to_pointcloud(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan const>& scan_msg,
    const std::shared_ptr<apollo::drivers::PointCloud>& point_cloud) {
  parser_->generate_pointcloud(scan_msg, point_cloud);

  if (point_cloud == nullptr || point_cloud->point_size() == 0) {
    AERROR << " point cloud has no point";
    return;
  }
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
