/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/software/module_monitor.h"

#include "absl/flags/flag.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

ABSL_FLAG(std::string, module_monitor_name, "ModuleMonitor",
          "Name of the modules monitor.");

ABSL_FLAG(double, module_monitor_interval, 1.5,
          "Process status checking interval in seconds.");

namespace apollo {
namespace monitor {

ModuleMonitor::ModuleMonitor()
    : RecurrentRunner(absl::GetFlag(FLAGS_module_monitor_name),
                      absl::GetFlag(FLAGS_module_monitor_interval)) {
  node_manager_ =
      cyber::service_discovery::TopologyManager::Instance()->node_manager();
}

void ModuleMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  const auto& mode = manager->GetHMIMode();

  // Check monitored components.
  auto* components = manager->GetStatus()->mutable_components();
  for (const auto& iter : mode.monitored_components()) {
    const std::string& name = iter.first;
    const auto& monitored_component = iter.second;
    if (monitored_component.has_module() &&
        apollo::common::util::ContainsKey(*components, name)) {
      const auto& config = monitored_component.module();
      auto* status = components->at(name).mutable_module_status();
      UpdateStatus(config, name, status);
    }
  }
}

void ModuleMonitor::UpdateStatus(
    const apollo::dreamview::ModuleMonitorConfig& config,
    const std::string& module_name, ComponentStatus* status) {
  status->clear_status();

  bool all_nodes_matched = true;
  for (const std::string& name : config.node_name()) {
    if (!node_manager_->HasNode(name)) {
      all_nodes_matched = false;
      break;
    }
  }
  if (all_nodes_matched) {
    // Working nodes are all matched. The module is running.
    SummaryMonitor::EscalateStatus(ComponentStatus::OK, module_name, status);
    return;
  }

  SummaryMonitor::EscalateStatus(ComponentStatus::FATAL, "", status);
}

}  // namespace monitor
}  // namespace apollo
