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

#ifndef MODULES_DREAMVIEW_BACKEND_HMI_HMI_STATUS_HELPER_H_
#define MODULES_DREAMVIEW_BACKEND_HMI_HMI_STATUS_HELPER_H_

#include <string>

#include "modules/dreamview/proto/hmi_status.pb.h"
#include "modules/monitor/proto/hardware_status.pb.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class HMIStatusHelper
 *
 * @brief Helper to report status to HMI.
 *        To use it, you must be able to publish to HMI_STATUS channel.
 */
class HMIStatusHelper {
 public:
  /*
   * @brief Report hardware status to HMI.
   * @param hardware_name the name of the hardware.
   * @param hw_status the status of the hardware.
   */
  static void ReportHardwareStatus(const std::string &hardware_name,
                                   const monitor::HardwareStatus &hw_status);

  /*
   * @brief Report module status to HMI.
   * @param module_name the name of the module.
   * @param module_status the status of the module.
   */
  static void ReportModuleStatus(const std::string &module_name,
                                 const ModuleStatus &module_status);
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_HMI_HMI_STATUS_HELPER_H_
