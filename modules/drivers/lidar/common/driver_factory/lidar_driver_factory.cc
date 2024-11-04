/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/common/driver_factory/lidar_driver_factory.h"

// #include "modules/drivers/lidar/hesai/driver/driver.h"
// #include "modules/drivers/lidar/rslidar/driver/driver.h"
#include "modules/drivers/lidar/velodyne/driver/driver.h"

namespace apollo {
namespace drivers {
namespace lidar {

LidarDriverFactory::LidarDriverFactory() {}
LidarDriverFactory::LidarDriverFactory(
    const apollo::drivers::lidar::config& config) {}
void LidarDriverFactory::RegisterLidarClients() {
//  Register(LidarParameter::HESAI,
//           [](const std::shared_ptr<::apollo::cyber::Node>& node,
//              const apollo::drivers::lidar::config& config) -> LidarDriver* {
//             return new hesai::HesaiDriver(node, config);
//           });
  //  Register(LidarParameter::ROBOSENSE,
  //           [](const std::shared_ptr<::apollo::cyber::Node>& node,
  //              const apollo::drivers::lidar::config& config) -> LidarDriver*
  //              {
  //             return new robosense::RobosenseDriver(node, config);
  //           });
  //
  Register(LidarParameter::VELODYNE,
           [](const std::shared_ptr<::apollo::cyber::Node>& node,
              const apollo::drivers::lidar::config& config) -> LidarDriver* {
             return velodyne::VelodyneDriverFactory::CreateDriver(
                 node, config.velodyne());
           });
}
std::unique_ptr<LidarDriver> LidarDriverFactory::CreateLidarDriver(
    const std::shared_ptr<::apollo::cyber::Node>& node,
    const apollo::drivers::lidar::config& parameter) {
  auto factory = CreateObject(parameter.brand(), node, parameter);
  if (!factory) {
    AERROR << "Failed to create lidar with parameter: "
           << parameter.DebugString();
  }
  return factory;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
