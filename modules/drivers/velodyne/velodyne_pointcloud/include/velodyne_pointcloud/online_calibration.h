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

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_ONLINE_CALIBRATION_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_ONLINE_CALIBRATION_H_

#include <ros/ros.h>
#include <vector>
#include "velodyne_msgs/VelodyneScanUnified.h"
#include "velodyne_pointcloud/calibration.h"

namespace apollo {
namespace drivers {
namespace velodyne {

constexpr double DEGRESS_TO_RADIANS = 3.1415926535897 / 180.0;

class OnlineCalibration {
 public:
  OnlineCalibration() {}
  ~OnlineCalibration() {}

  int decode(const velodyne_msgs::VelodyneScanUnified::ConstPtr& scan_msgs);
  void dump(const std::string& file_path);
  void get_unit_index();
  bool inited() const {
    return inited_;
  }
  Calibration calibration() const {
    return calibration_;
  }

 private:
  bool inited_;
  Calibration calibration_;
  std::vector<uint8_t> status_types_;
  std::vector<uint8_t> status_values_;
  // store first two "unit#" value index
  std::vector<int> unit_indexs_;
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_POINTCLOUD_ONLINE_CALIBRATION_H_