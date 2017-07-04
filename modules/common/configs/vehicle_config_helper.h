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

#ifndef MODULES_CONFIGS_VEHICLE_CONFIG_H_
#define MODULES_CONFIGS_VEHICLE_CONFIG_H_

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/macro.h"

/**
 * @namespace apollo::common::config
 * @brief apollo::common::config
 */
namespace apollo {
namespace common {
namespace config {

/**
 * @class VehicleConfigHelper
 *
 * @Brief This is a helper class that can load vehicle configurations. The
 * vehicle configurations are
 * defined modules/common/configs/proto/vehicle_config.proto
 */
class VehicleConfigHelper {
 public:
  /**
   * @brief Initialize vehicle configurations with default configuration file
   * pointed by gflags FLAGS_vehicle_config_path. The code will crash if
   * FLAGS_vehicle_config_path does not exit or it points to a file with invalid
   * format.
   */
  static void Init();

  /**
   * @brief Initialize vehicle configurations with \p config
   * @param config A VehicleConfig class instance. The VehicleConfig class is
   * defined by modules/common/configs/proto/vehicle_config.proto.
   */
  static void Init(const VehicleConfig& config);

  /**
   * @brief Initialize vehicle configurations with \p config_file.
   * The code will crash if \p config_file does not exist or \p config_file has
   * invalid format.
   * @param config_file The configuration file path. The format of the file is
   * defined by protobuf file
   * modules/common/configs/proto/vehicle_config.proto.
   */
  static void Init(const std::string& config_file);

  /**
   * @brief Get the current vehicle configuration.
   * @return the current VehicleConfig instance reference.
   */
  static const VehicleConfig& GetConfig();

 private:
  static VehicleConfig vehicle_config_;
  DECLARE_SINGLETON(VehicleConfigHelper);
};

}  // namespace config
}  // namespace common
}  // namespace apollo

#endif  // MODULES_CONFIGS_VEHICLE_CONFIG_H_
