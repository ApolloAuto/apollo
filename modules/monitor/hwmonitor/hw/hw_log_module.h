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

#ifndef MODULES_PLATFORM_HW_LOG_MODULE_H_
#define MODULES_PLATFORM_HW_LOG_MODULE_H_

#include "modules/monitor/common/annotations.h"
#include "modules/monitor/common/log.h"

/**
 * @namespace apollo::platform::hw
 * @brief apollo::platform::hw
 */
namespace apollo {
namespace platform {
namespace hw {

// Shared HW log module; each component has the option to use this shared module
// or use its own.
extern log::LogModule _shrd_hw_log_mod PRIVATE;

/// Get pointer to the shared HW log module.
inline log::LogModule *get_log_module() {
  return &_shrd_hw_log_mod;
}

/// Configs the shared HW log module
/// @param log_lvl log level
/// @param dbg_lvl debug level
/// @param log_fn log writer function
void config_log(int log_lvl, int dbg_lvl, log::LogFn *log_fn = nullptr);

}  // namespace hw
}  // namespace platform
}  // namespace apollo

#endif  // MODULES_PLATFORM_HW_LOG_MODULE_H_
