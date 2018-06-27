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

#ifndef MODULES_DRIVERS_LIDAR_VELODYN_LIDAR_VELODYNE_H_
#define MODULES_DRIVERS_LIDAR_VELODYN_LIDAR_VELODYNE_H_

#include <string>

#include "modules/common/apollo_app.h"

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
class LidarVelodyne : public apollo::common::ApolloApp {
 public:
  LidarVelodyne() = default;
  virtual ~LidarVelodyne() = default;

  std::string Name() const override;
  common::Status Init() override;
  common::Status Start() override;
  void Stop() override;

 private:
};

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_LIDAR_VELODYN_LIDAR_VELODYNE_H_
