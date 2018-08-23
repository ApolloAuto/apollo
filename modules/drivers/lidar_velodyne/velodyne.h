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

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_H_

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ros/include/sensor_msgs/PointCloud2.h"
#include "ros/include/velodyne_msgs/VelodyneScanUnified.h"

#include "modules/common/apollo_app.h"
#include "modules/common/util/util.h"
#include "modules/drivers/lidar_velodyne/driver/driver_nodelet.h"

/**
 * @namespace apollo::velodyne
 * @brief apollo::velodyne
 */
namespace apollo {
namespace drivers {
namespace lidar_velodyne {

/**
 * @class Velodyne
 *
 * @brief velodyne module main class
 */
class Velodyne : public apollo::common::ApolloApp {
 public:
  Velodyne() = default;
  virtual ~Velodyne() = default;

  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  bool running_ = true;
  std::vector<std::shared_ptr<std::thread> > threads_;

  DriverNodelet driver_nodelet_;
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_H_
