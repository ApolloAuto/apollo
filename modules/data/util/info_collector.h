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

#ifndef MODULES_DATA_UTIL_INFO_COLLECTOR_H_
#define MODULES_DATA_UTIL_INFO_COLLECTOR_H_

#include <string>

#include "modules/common/macro.h"
#include "modules/data/proto/static_info.pb.h"

/**
 * @namespace apollo::data
 * @brief apollo::data
 */
namespace apollo {
namespace data {

class InfoCollector {
 public:
  // Get task information.
  static const StaticInfo &GetStaticInfo();

  // Get specific information.
  // Listening topics: ChassisDetail.
  static const VehicleInfo &GetVehicleInfo();
  static const EnvironmentInfo &GetEnvironmentInfo();
  static const HardwareInfo &GetHardwareInfo();
  static const SoftwareInfo &GetSoftwareInfo();
  static const UserInfo &GetUserInfo();

  // Utils.
  static std::string GetDockerImage();

 private:
  StaticInfo static_info_;
  StaticInfoConf config_;

  DECLARE_SINGLETON(InfoCollector);
};

}  // namespace data
}  // namespace apollo

#endif  // MODULES_DATA_UTIL_INFO_COLLECTOR_H_
