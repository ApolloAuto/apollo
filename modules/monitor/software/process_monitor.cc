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

#include <unordered_set>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"

DEFINE_string(process_monitor_name, "ProcessMonitor",
              "Name of the process monitor.");
DEFINE_double(process_monitor_interval, 1.5,
              "Process status checking interval (s).");

DEFINE_string(module_monitor_conf_path,
              "modules/monitor/conf/module_monitor_conf.pb.txt",
              "Path of the module monitor config file.");

namespace apollo {
namespace monitor {

using apollo::common::util::GetProtoFromFile;

ProcessMonitor::ProcessMonitor(SystemStatus *system_status)
    : RecurrentRunner(FLAGS_process_monitor_name,
                      FLAGS_process_monitor_interval)
    , status_(system_status->mutable_modules()) {
  CHECK(GetProtoFromFile(FLAGS_module_monitor_conf_path, &config_))
        << "Unable to parse config file " << FLAGS_module_monitor_conf_path;

  // Init module status if it has set binary to check.
  for (const auto &module_conf : config_.modules()) {
    if (!module_conf.process_cmd_keywords().empty()) {
      status_->insert({module_conf.name(), {}});
    }
  }
}

void ProcessMonitor::RunOnce(const double current_time) {
  // Set all processes as not-running by default.
  for (const auto &module_conf : config_.modules()) {
    if (!module_conf.process_cmd_keywords().empty()) {
      (*status_)[module_conf.name()].set_process_running(false);
    }
  }

  const auto procs = common::util::ListSubDirectories("/proc");
  for (const auto &proc : procs) {
    // Get process command string.
    std::string cmd_string;
    const auto cmd_file = common::util::StrCat("/proc/", proc, "/cmdline");
    if (!common::util::GetContent(cmd_file, &cmd_string)) {
      continue;
    }

    for (const auto &module_conf : config_.modules()) {
      const auto &keywords = module_conf.process_cmd_keywords();
      // Check if the command string contains all keywords of this module.
      const bool keywords_matched = !keywords.empty() && std::find_if(
          keywords.begin(), keywords.end(),
          [cmd_string](const std::string &keyword)->bool {
            return cmd_string.find(keyword) == std::string::npos;
          }) == keywords.end();

      if (keywords_matched) {
        (*status_)[module_conf.name()].set_process_running(true);
        ADEBUG << "Found " << module_conf.name() << " module "
                  "is running on process " << proc;
        break;
      }
    }
  }
}

}  // namespace monitor
}  // namespace apollo
