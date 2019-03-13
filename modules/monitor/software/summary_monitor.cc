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

#include "modules/monitor/software/summary_monitor.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(summary_monitor_name, "SummaryMonitor",
              "Name of the summary monitor.");

DEFINE_double(system_status_publish_interval, 10,
              "SystemStatus publish interval.");

namespace apollo {
namespace monitor {

void SummaryMonitor::EscalateStatus(const ComponentStatus::Status new_status,
                                    const std::string& message,
                                    ComponentStatus* current_status) {
  // Overwrite priority: FATAL > ERROR > WARN > OK > UNKNOWN.
  if (new_status > current_status->status()) {
    current_status->set_status(new_status);
    if (!message.empty()) {
      current_status->set_message(message);
    } else {
      current_status->clear_message();
    }
  }
}

// Set interval to 0, so it runs every time when ticking.
SummaryMonitor::SummaryMonitor()
    : RecurrentRunner(FLAGS_summary_monitor_name, 0) {}

void SummaryMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  auto* status = manager->GetStatus();
  // Escalate the summary status to the most severe one.
  for (auto& component : *status->mutable_components()) {
    auto* summary = component.second.mutable_summary();
    const auto& process_status = component.second.process_status();
    EscalateStatus(process_status.status(), process_status.message(), summary);
    const auto& channel_status = component.second.channel_status();
    EscalateStatus(channel_status.status(), channel_status.message(), summary);
    const auto& resource_status = component.second.resource_status();
    EscalateStatus(resource_status.status(), resource_status.message(),
                   summary);
    const auto& other_status = component.second.other_status();
    EscalateStatus(other_status.status(), other_status.message(), summary);
  }

  // Get fingerprint of current status.
  // Don't use DebugString() which has known bug on Map field. The string
  // doesn't change though the value has changed.
  static std::hash<std::string> hash_fn;
  std::string proto_bytes;
  status->SerializeToString(&proto_bytes);
  const size_t new_fp = hash_fn(proto_bytes);

  if (system_status_fp_ != new_fp ||
      current_time - last_broadcast_ > FLAGS_system_status_publish_interval) {
    static auto writer =
        manager->CreateWriter<SystemStatus>(FLAGS_system_status_topic);

    apollo::common::util::FillHeader("SystemMonitor", status);
    writer->Write(*status);
    status->clear_header();
    system_status_fp_ = new_fp;
    last_broadcast_ = current_time;
  }
}

}  // namespace monitor
}  // namespace apollo
