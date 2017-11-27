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

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/string_util.h"

DEFINE_string(summary_monitor_name, "SummaryMonitor",
              "Name of the summary monitor.");

namespace apollo {
namespace monitor {
namespace {

using apollo::common::adapter::AdapterBase;
using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::StrCat;
using apollo::common::util::StringPrintf;

}  // namespace

// Set interval to 0, so it runs every time when ticking.
SummaryMonitor::SummaryMonitor()
    : RecurrentRunner(FLAGS_summary_monitor_name, 0) {
  CHECK(AdapterManager::GetSystemStatus())
      << "SystemStatusAdapter is not initialized.";
}

void SummaryMonitor::RunOnce(const double current_time) {
  SummarizeModules();
  SummarizeHardware();
  // Get fingerprint of current status.
  // Don't use DebugString() which has known bug on Map field. The string
  // doesn't change though the value has changed.
  static std::hash<std::string> hash_fn;
  std::string proto_bytes;
  auto *system_status = MonitorManager::GetStatus();
  system_status->clear_header();
  system_status->SerializeToString(&proto_bytes);
  const size_t new_fp = hash_fn(proto_bytes);

  if (system_status_fp_ != new_fp) {
    AdapterManager::FillSystemStatusHeader("SystemMonitor", system_status);
    AdapterManager::PublishSystemStatus(*system_status);
    ADEBUG << "Published system status: " << system_status->DebugString();
    system_status_fp_ = new_fp;
  }
}

void SummaryMonitor::SummarizeModules() {
  for (auto &module : *MonitorManager::GetStatus()->mutable_modules()) {
    auto &status = module.second;
    status.set_summary(Summary::UNKNOWN);
    status.clear_msg();

    if (status.has_process_status() && !status.process_status().running()) {
      status.set_summary(Summary::FATAL);
      status.set_msg("No process");
      continue;
    }

    if (status.has_topic_status()) {
      if (!status.topic_status().has_message_delay()) {
        status.set_summary(Summary::OK);
        continue;
      }

      const double delay = status.topic_status().message_delay();
      if (delay < 0) {
        status.set_summary(Summary::ERROR);
        status.set_msg("No message");
      } else {
        status.set_summary(Summary::WARN);
        status.set_msg("Notable delay");
      }
    }
  }
}

void SummaryMonitor::SummarizeHardware() {
  for (auto &hardware : *MonitorManager::GetStatus()->mutable_hardware()) {
    auto &status = hardware.second;
    status.set_summary(Summary::UNKNOWN);
    status.clear_msg();

    // If we don't have the status, keeps as UNKNOWN.
    if (status.has_status()) {
      switch (status.status()) {
        case HardwareStatus::NOT_PRESENT:
          status.set_summary(Summary::FATAL);
          break;
        case HardwareStatus::NOT_READY:
          status.set_summary(Summary::WARN);
          break;
        case HardwareStatus::OK:
          status.set_summary(Summary::OK);
          break;
        default:
          status.set_summary(Summary::ERROR);
          break;
      }
    }

    if (status.summary() == Summary::OK && status.has_topic_status()) {
      if (!status.topic_status().has_message_delay()) {
        continue;
      }

      const double delay = status.topic_status().message_delay();
      if (delay < 0) {
        status.set_summary(Summary::ERROR);
        status.set_msg("No message");
      } else {
        status.set_summary(Summary::WARN);
        status.set_msg("Notable delay");
      }
    }
  }
}

}  // namespace monitor
}  // namespace apollo
