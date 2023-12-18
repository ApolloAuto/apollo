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

#include "modules/monitor/software/process_monitor.h"

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(process_monitor_name, "ProcessMonitor",
              "Name of the process monitor.");

DEFINE_double(process_monitor_interval, 1.5,
              "Process status checking interval in seconds.");

namespace apollo {
namespace monitor {

ProcessMonitor::ProcessMonitor()
    : RecurrentRunner(FLAGS_process_monitor_name,
                      FLAGS_process_monitor_interval) {}

void ProcessMonitor::RunOnce(const double current_time) {
  // Get running processes.
  std::vector<std::string> running_processes;
  for (const auto& cmd_file : cyber::common::Glob("/proc/*/cmdline")) {
    // Get process command string.
    std::string cmd_string;
    if (cyber::common::GetContent(cmd_file, &cmd_string) &&
        !cmd_string.empty()) {
      // In /proc/<PID>/cmdline, the parts are separated with \0, which will be
      // converted back to whitespaces here.
      std::replace(cmd_string.begin(), cmd_string.end(), '\0', ' ');
      running_processes.push_back(cmd_string);
    }
  }

  auto manager = MonitorManager::Instance();
  const auto& mode = manager->GetHMIMode();

  // Check HMI modules.
  auto* hmi_modules = manager->GetStatus()->mutable_hmi_modules();
  for (const auto& iter : mode.modules()) {
    const std::string& module_name = iter.first;
    const auto& config = iter.second.process_monitor_config();
    UpdateStatus(running_processes, config, &hmi_modules->at(module_name));
  }

  // Check monitored components.
  auto* components = manager->GetStatus()->mutable_components();
  for (const auto& iter : mode.monitored_components()) {
    const std::string& name = iter.first;
    if (iter.second.has_process() &&
        apollo::common::util::ContainsKey(*components, name)) {
      const auto& config = iter.second.process();
      auto* status = components->at(name).mutable_process_status();
      UpdateStatus(running_processes, config, status);
    }
  }

  // Check other components.
  auto* other_components = manager->GetStatus()->mutable_other_components();
  for (const auto& iter : mode.other_components()) {
    const std::string& name = iter.first;
    const auto& config = iter.second;
    UpdateStatus(running_processes, config, &other_components->at(name));
  }

  // Check global components.
  auto* global_components = manager->GetStatus()->mutable_global_components();
  for (const auto& iter : mode.global_components()) {
    const std::string& name = iter.first;
    if (iter.second.has_process() &&
        apollo::common::util::ContainsKey(*global_components, name)) {
      const auto& config = iter.second.process();
      auto* status = global_components->at(name).mutable_process_status();
      UpdateStatus(running_processes, config, status);
    }
  }
}

void ProcessMonitor::UpdateStatus(
    const std::vector<std::string>& running_processes,
    const apollo::dreamview::ProcessMonitorConfig& config,
    ComponentStatus* status) {
  status->clear_status();
  for (const std::string& command : running_processes) {
    bool all_keywords_matched = true;
    for (const std::string& keyword : config.command_keywords()) {
      if (command.find(keyword) == std::string::npos) {
        all_keywords_matched = false;
        break;
      }
    }
    if (all_keywords_matched) {
      // Process command keywords are all matched. The process is running.
      SummaryMonitor::EscalateStatus(ComponentStatus::OK, command, status);
      return;
    }
  }
  SummaryMonitor::EscalateStatus(ComponentStatus::FATAL, "", status);
}

}  // namespace monitor
}  // namespace apollo
