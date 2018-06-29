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
#include "modules/monitor/monitor.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/hardware/can/can_monitor.h"
#include "modules/monitor/hardware/gps/gps_monitor.h"
#include "modules/monitor/reporters/static_info_reporter.h"
#include "modules/monitor/reporters/vehicle_state_reporter.h"
#include "modules/monitor/software/process_monitor.h"
#include "modules/monitor/software/summary_monitor.h"
#include "modules/monitor/software/topic_monitor.h"

DEFINE_string(monitor_adapter_config_filename,
              "modules/monitor/conf/adapter.conf",
              "Directory which contains a group of related maps.");

DEFINE_double(monitor_running_interval, 0.5, "Monitor running interval.");

namespace apollo {
namespace monitor {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using std::make_unique;

Monitor::Monitor() : monitor_thread_(FLAGS_monitor_running_interval) {
}

Status Monitor::Init() {
  AdapterManager::Init(FLAGS_monitor_adapter_config_filename);

  monitor_thread_.RegisterRunner(make_unique<CanMonitor>());
  monitor_thread_.RegisterRunner(make_unique<GpsMonitor>());
  monitor_thread_.RegisterRunner(make_unique<ProcessMonitor>());

  const auto &config = MonitorManager::GetConfig();
  for (const auto &module : config.modules()) {
    if (module.has_topic_conf()) {
      auto *module_status = MonitorManager::GetModuleStatus(module.name());
      monitor_thread_.RegisterRunner(make_unique<TopicMonitor>(
          module.topic_conf(), module_status->mutable_topic_status()));
    }
  }
  for (const auto &hardware : config.hardware()) {
    if (hardware.has_topic_conf()) {
      auto *hw_status = MonitorManager::GetHardwareStatus(hardware.name());
      monitor_thread_.RegisterRunner(make_unique<TopicMonitor>(
          hardware.topic_conf(), hw_status->mutable_topic_status()));
    }
  }

  // Register online reporters.
  if (MonitorManager::GetConfig().has_online_report_endpoint()) {
    monitor_thread_.RegisterRunner(make_unique<VehicleStateReporter>());
  }
  // Register StaticInfo reporter.
  monitor_thread_.RegisterRunner(make_unique<StaticInfoReporter>());

  // Register the SummaryMonitor as last runner, so it will monitor all changes
  // made by the previous runners.
  monitor_thread_.RegisterRunner(make_unique<SummaryMonitor>());
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
