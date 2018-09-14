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

#include "modules/common/time/time.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/hardware/gps/gps_monitor.h"
#include "modules/monitor/hardware/resource_monitor.h"
#include "modules/monitor/reporters/static_info_reporter.h"
#include "modules/monitor/software/localization_monitor.h"
#include "modules/monitor/software/process_monitor.h"
#include "modules/monitor/software/summary_monitor.h"
// #include "modules/monitor/software/topic_monitor.h"

DEFINE_double(monitor_running_interval, 0.5, "Monitor running interval.");

namespace apollo {
namespace monitor {

bool Monitor::Init() {
  MonitorManager::Init(node_);

  // TODO(xiaoxq): Migrate CAN monitor and topic monitor.
  // runners_.emplace_back(new CanMonitor());
  runners_.emplace_back(new GpsMonitor());
  runners_.emplace_back(new ProcessMonitor());

  const auto &config = MonitorManager::GetConfig();

  // for (const auto &module : config.modules()) {
  //   if (module.has_topic_conf()) {
  //     auto *module_status = MonitorManager::GetModuleStatus(module.name());
  //     runners_.emplace_back(new TopicMonitor(
  //         module.topic_conf(), module_status->mutable_topic_status()));
  //   }
  // }
  // for (const auto &hardware : config.hardware()) {
  //   if (hardware.has_topic_conf()) {
  //     auto *hw_status = MonitorManager::GetHardwareStatus(hardware.name());
  //     runners_.emplace_back(new TopicMonitor(
  //         hardware.topic_conf(), hw_status->mutable_topic_status()));
  //   }
  // }

  // Register resource monitor.
  runners_.emplace_back(new ResourceMonitor(config.resource_conf()));

  // Register StaticInfo reporter.
  runners_.emplace_back(new StaticInfoReporter());

  // Register the SummaryMonitor as last runner, so it will monitor all changes
  // made by the previous runners.
  runners_.emplace_back(new SummaryMonitor());
  return true;
}

bool Monitor::Proc() {
  const double current_time = apollo::common::time::Clock::NowInSeconds();
  MonitorManager::InitFrame(current_time);
  for (auto &runner : runners_) {
    runner->Tick(current_time);
  }
  return true;
}

}  // namespace monitor
}  // namespace apollo
