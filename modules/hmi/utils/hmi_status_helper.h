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
 * @file hmi_status_helper.h
 * @brief the class of HMIStatusHelper
 */

#ifndef MODULES_HMI_UTILS_HMI_STATUS_HELPER_H_
#define MODULES_HMI_UTILS_HMI_STATUS_HELPER_H_

#include <vector>

#include "modules/hmi/proto/runtime_status.pb.h"

/**
 * @namespace apollo::tools
 * @brief apollo::tools
 */
namespace apollo {
namespace hmi {

/**
 * @class HMIStatusHelper
 *
 * @brief Helper to report status to HMI.
 */
class HMIStatusHelper {
 public:
  /*
   * @brief Report hardware status to HMI.
   * @param hardware_status the vector of hardware status
   */
  static void ReportHardwareStatus(
      const std::vector<HardwareStatus>& hardware_status);

  /*
   * @brief Report module status to HMI.
   * @param module_status the status of the module
   */
  static void ReportModuleStatus(const ModuleStatus& module_status);
};

}  // namespace hmi
}  // namespace apollo

#endif  // MODULES_HMI_UTILS_HMI_STATUS_HELPER_H_
