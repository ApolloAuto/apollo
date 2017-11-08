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
#include "modules/common/adapters/adapter_manager.h"
#include "modules/monitor/hardware/can/can_monitor.h"
#include "modules/monitor/hardware/gps/gps_monitor.h"
#include "modules/monitor/monitor.h"
#include "modules/monitor/software/process_monitor.h"

DEFINE_string(monitor_adapter_config_filename,
              "modules/monitor/conf/adapter.conf",
              "Directory which contains a group of related maps.");

DEFINE_double(monitor_running_interval, 0.5, "Monitor running interval.");

namespace apollo {
namespace monitor {
namespace {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::make_unique;

// A runner which monitors if the system status has changed.
class StatusChangeMonitor : public RecurrentRunner {
 public:
  // Set internal to 0, so it runs every time when ticking.
  explicit StatusChangeMonitor(SystemStatus *system_status)
      : RecurrentRunner("SystemStatusMonitor", 0)
      , system_status_(system_status) {
  }

  // Publish the new system status if it changed.
  void RunOnce(const double current_time) override {
    // Get fingerprint of current status.
    // Don't use DebugString() which has known bug on Map field. The string
    // doesn't change though the value has changed.
    static std::hash<std::string> hash_fn;
    std::string proto_bytes;
    system_status_->clear_header();
    system_status_->SerializeToString(&proto_bytes);
    const size_t new_fp = hash_fn(proto_bytes);

    if (system_status_fp_ != new_fp) {
      AdapterManager::FillSystemStatusHeader(name_, system_status_);
      AdapterManager::PublishSystemStatus(*system_status_);
      ADEBUG << "Published system status: " << system_status_->DebugString();
      system_status_fp_ = new_fp;
    }
  }

 private:
  SystemStatus *system_status_;
  size_t system_status_fp_ = 0;
};

}  // namespace

Monitor::Monitor()
    : monitor_thread_(FLAGS_monitor_running_interval) {
}

Status Monitor::Init() {
  AdapterManager::Init(FLAGS_monitor_adapter_config_filename);

  // Check the expected adapters are initialized.
  CHECK(AdapterManager::GetGnssStatus()) <<
      "GnssStatusAdapter is not initialized.";
  CHECK(AdapterManager::GetInsStatus()) <<
      "InsStatusAdapter is not initialized.";
  CHECK(AdapterManager::GetSystemStatus()) <<
      "SystemStatusAdapter is not initialized.";

  monitor_thread_.RegisterRunner(make_unique<CanMonitor>(&system_status_));
  monitor_thread_.RegisterRunner(make_unique<GpsMonitor>(&system_status_));
  monitor_thread_.RegisterRunner(make_unique<ProcessMonitor>(&system_status_));

  // Register the StatusChangeMonitor as last runner, so it will monitor all
  // changes made by the previous runners.
  monitor_thread_.RegisterRunner(
      make_unique<StatusChangeMonitor>(&system_status_));
  return Status::OK();
}

Status Monitor::Start() {
  monitor_thread_.Start();
  return Status::OK();
}

void Monitor::Stop() {
  monitor_thread_.Stop();
}

}  // namespace monitor
}  // namespace apollo
