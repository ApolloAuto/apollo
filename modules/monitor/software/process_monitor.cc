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
#include "modules/common/log.h"
#include "modules/common/util/file.h"

DEFINE_string(process_monitor_name, "ProcessMonitor",
              "Name of the process monitor.");

DEFINE_double(process_monitor_interval, 1.5,
              "Process status checking interval (s).");

namespace apollo {
namespace monitor {
namespace {

template <class Iterable>
bool ContainsAll(const std::string &full, const Iterable &parts) {
  for (const auto &part : parts) {
    if (full.find(part) == std::string::npos) {
      return false;
    }
  }
  return true;
}

}  // namespace

ProcessMonitor::ProcessMonitor()
    : RecurrentRunner(FLAGS_process_monitor_name,
                      FLAGS_process_monitor_interval) {
}

void ProcessMonitor::RunOnce(const double current_time) {
  // Get running processes.
  std::map<std::string, std::string> running_processes;
  const auto procs = common::util::ListSubDirectories("/proc");
  for (const auto &proc : procs) {
    // Get process command string.
    std::string cmd_string;
    const auto cmd_file = common::util::StrCat("/proc/", proc, "/cmdline");
    if (common::util::GetContent(cmd_file, &cmd_string)) {
      running_processes.emplace(proc, cmd_string);
    }
  }

  for (const auto &module : MonitorManager::GetConfig().modules()) {
    if (module.has_process_conf()) {
      UpdateModule(module.name(), module.process_conf(), running_processes);
    }
  }
}

void ProcessMonitor::UpdateModule(
    const std::string &module_name, const ProcessConf &config,
    const std::map<std::string, std::string> &running_processes) {
  auto *status = MonitorManager::GetModuleStatus(module_name);
  for (const auto &proc : running_processes) {
    if (ContainsAll(proc.second, config.process_cmd_keywords())) {
      status->mutable_process_status()->set_running(true);
      ADEBUG << "Module " << module_name
             << " is running on process " << proc.first;
      return;
    }
  }
  status->mutable_process_status()->set_running(false);
}

}  // namespace monitor
}  // namespace apollo
