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

/**
 * @file
 * @brief Defines the LidarFactory class.
 */

#pragma once

#include <memory>
#include <unordered_map>

#include "modules/drivers/lidar/proto/config.pb.h"
#include "modules/drivers/lidar/proto/lidar_parameter.pb.h"

#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "modules/common/util/factory.h"
#include "modules/drivers/lidar/common/driver_factory/driver_base.h"
// #include "modules/drivers/lidar/hesai/parser/parser.h"

/**
 * @namespace apollo::drivers::lidar
 *
 */

namespace apollo {
namespace drivers {
namespace lidar {

class LidarDriverFactory
    : public apollo::common::util::Factory<
          LidarParameter::LidarBrand, LidarDriver,
          LidarDriver* (*)(const std::shared_ptr<::apollo::cyber::Node>& node,
                           const apollo::drivers::lidar::config& config)> {
 public:
  LidarDriverFactory(const apollo::drivers::lidar::config& config);
  /**
   * @brief Register the lidar driver of all brands. This function call the
   *        Function apollo::common::util::Factory::Register() for all of the
   *        lidar.
   */
  void RegisterLidarClients();

  /**
   * @brief Create a pointer to a specified brand of lidar. The brand is
   *        set in the parameter.
   * @param parameter The parameter to create the CAN client.
   * @return A pointer to the created CAN client.
   */
  std::unique_ptr<LidarDriver> CreateLidarDriver(
      const std::shared_ptr<::apollo::cyber::Node>& node,
      const apollo::drivers::lidar::config& parameter);

 protected:
 private:
  DECLARE_SINGLETON(LidarDriverFactory)
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
