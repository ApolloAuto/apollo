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

#ifndef ONBOARD_DRIVERS_SUTENG_INCLUDE_SUTENG_PARSER_CONVERT_H
#define ONBOARD_DRIVERS_SUTENG_INCLUDE_SUTENG_PARSER_CONVERT_H

#include <memory>

#include "modules/drivers/lidar/lidar_robosense/proto/sensor_suteng.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/lidar/lidar_robosense/include/parser/robosense_parser.h"

namespace autobot {
namespace drivers {
namespace robosense {

// convert suteng data to pointcloud and republish
class Convert {
 public:
  explicit Convert(const cybertron::proto::SutengConfig& robo_config);
  ~Convert();

  void convert_velodyne_to_pointcloud(
      const std::shared_ptr<adu::common::sensor::suteng::SutengScan const>&
          scan_msg,
     const  std::shared_ptr<apollo::drivers::PointCloud>& point_cloud);

  bool Init();
  uint32_t GetPointSize();

 private:
  RobosenseParser* _parser;
  cybertron::proto::SutengConfig _config;
};

}  // namespace robosense
}  // namespace drivers
}  // namespace autobot

#endif  // ONBOARD_DRIVERS_SUTENG_INCLUDE_SUTENG_PARSER_CONVERT_H
