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

#ifndef MODULES_MONITOR_HARDWARE_CAN_ESDCAN_ESDCAN_ERR_STR_H_
#define MODULES_MONITOR_HARDWARE_CAN_ESDCAN_ESDCAN_ERR_STR_H_

#include "esd_can/include/ntcan.h"

/**
 * @namespace apollo::monitor::hw
 * @brief apollo::monitor::hw
 */
namespace apollo {
namespace monitor {
namespace hw {

/// Returns an error meesage string for the given status code.
const char *esdcan_err_to_str(NTCAN_RESULT ntstatus);

}  // namespace hw
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_HARDWARE_CAN_ESDCAN_ESDCAN_ERR_STR_H_
