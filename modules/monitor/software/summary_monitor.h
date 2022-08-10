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

#include "modules/monitor/common/recurrent_runner.h"
#include "modules/common_msgs/monitor_msgs/system_status.pb.h"

namespace apollo {
namespace monitor {

// A monitor which summarize other monitors' result and publish the whole status
// if it has changed.
class SummaryMonitor : public RecurrentRunner {
 public:
  SummaryMonitor();
  void RunOnce(const double current_time) override;

  // Escalate the status to a higher priority new status:
  //    FATAL > ERROR > WARN > OK > UNKNOWN.
  static void EscalateStatus(const ComponentStatus::Status new_status,
                             const std::string& message,
                             ComponentStatus* current_status);

 private:
  size_t system_status_fp_ = 0;
  double last_broadcast_ = 0;
};

}  // namespace monitor
}  // namespace apollo
