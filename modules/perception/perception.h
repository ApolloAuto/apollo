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

/**
 * @file
 */

#ifndef MODEULES_PERCEPTION_PERCEPTION_H_
#define MODEULES_PERCEPTION_PERCEPTION_H_

#include <memory>
#include <string>

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/perception/obstacle/onboard/lidar_process.h"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

/**
 * @namespace apollo::perception
 * @brief apollo::perception
 */
namespace apollo {
namespace perception {

class Perception : public common::ApolloApp {
 public:
  std::string Name() const override;
  common::Status Init() override;
  common::Status Start() override;
  void Stop() override;

  // Upon receiving point cloud data
  void OnPointCloud(const sensor_msgs::PointCloud2& message);

 private:
  std::unique_ptr<LidarProcess> lidar_process_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_PERCEPTION_H_
