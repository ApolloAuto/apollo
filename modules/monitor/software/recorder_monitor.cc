/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/software/recorder_monitor.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/data/tools/smart_recorder/proto/smart_recorder_status.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(smart_recorder_monitor_name, "SmartRecorderMonitor",
              "Name of the smart recorder monitor.");
DEFINE_double(smart_recorder_monitor_interval, 5,
              "Smart recorder status checking interval (s).");
DEFINE_string(smart_recorder_component_name, "SmartRecorder",
              "Smart recorder component name.");

namespace apollo {
namespace monitor {

using apollo::common::util::StrCat;
using apollo::data::RecordingState;
using apollo::data::SmartRecorderStatus;

RecorderMonitor::RecorderMonitor()
    : RecurrentRunner(FLAGS_smart_recorder_monitor_name,
                      FLAGS_smart_recorder_monitor_interval) {}

void RecorderMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  auto* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(),
      FLAGS_smart_recorder_component_name);
  if (component == nullptr) {
    // SmartRecorder is not monitored in current mode, skip.
    return;
  }

  static auto reader =
      manager->CreateReader<SmartRecorderStatus>(FLAGS_recorder_status_topic);
  reader->Observe();
  const auto status = reader->GetLatestObserved();

  ComponentStatus* component_status = component->mutable_other_status();
  component_status->clear_status();
  if (status == nullptr) {
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                   "No SmartRecorderStatus received",
                                   component_status);
    return;
  }

  // Translate SmartRecorderStatus to ComponentStatus. Note that ERROR and FATAL
  // will trigger safety mode in current settings.
  switch (status->recording_state()) {
    case RecordingState::RECORDING:
      SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", component_status);
      break;
    case RecordingState::TERMINATING:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN, StrCat("WARNNING: ", status->state_message()),
          component_status);
      break;
    case RecordingState::STOPPED:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::OK, StrCat("STOPPED: ", status->state_message()),
          component_status);
      break;
    default:
      AFATAL << "Unknown recording status: " << status->recording_state();
      break;
  }
}

}  // namespace monitor
}  // namespace apollo
