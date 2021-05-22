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
 * @brief Defines the CanFrame struct and CanClient interface.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/cyber.h"
/**
 * @namespace apollo::drivers::lidar
 * @brief apollo::drivers::lidar
 */
namespace apollo {
namespace drivers {
namespace lidar {

/**
 * @class LidarDriver
 * @brief The class which defines the lidar driver .
 */
class LidarDriver {
 public:
  /**
   * @brief Constructor
   */
  LidarDriver() {}
  explicit LidarDriver(const std::shared_ptr<::apollo::cyber::Node>& node)
      : node_(node) {}

  /**
   * @brief Destructor
   */
  virtual ~LidarDriver() = default;

  /**
   * @brief Initialize the lidar driver.
   * @return If the initialization is successful.
   */
  virtual bool Init() = 0;

 protected:
  std::shared_ptr<cyber::Node> node_;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
