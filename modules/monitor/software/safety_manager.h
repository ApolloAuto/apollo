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
#ifndef MODULES_MONITOR_SOFTWARE_SAFETY_MANAGER_H_
#define MODULES_MONITOR_SOFTWARE_SAFETY_MANAGER_H_

#include <string>

#include "modules/dreamview/proto/hmi_config.pb.h"

namespace apollo {
namespace monitor {

// Check if we need to switch to safety mode, and then
// 1. Notify driver to take action.
// 2. Trigger EStop if no proper action was taken.
class SafetyManager {
 public:
  SafetyManager();
  void CheckSafety(const double current_time);

 private:
  bool ShouldTriggerSafeMode(const double current_time);

  apollo::dreamview::HMIConfig hmi_config_;
};

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_SOFTWARE_SAFETY_MANAGER_H_
