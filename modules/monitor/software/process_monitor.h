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
#pragma once

#include <string>
#include <vector>

#include "modules/common_msgs/dreamview_msgs/hmi_mode.pb.h"
#include "modules/common_msgs/monitor_msgs/system_status.pb.h"

#include "modules/monitor/common/recurrent_runner.h"

namespace apollo {
namespace monitor {

class ProcessMonitor : public RecurrentRunner {
 public:
  ProcessMonitor();
  void RunOnce(const double current_time) override;

 private:
  static void UpdateStatus(
      const std::vector<std::string>& running_processes,
      const apollo::dreamview::ProcessMonitorConfig& config,
      ComponentStatus* status);
};

}  // namespace monitor
}  // namespace apollo
