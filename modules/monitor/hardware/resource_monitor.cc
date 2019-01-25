/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/hardware/resource_monitor.h"

#include <string>

#include "boost/filesystem.hpp"
#include "cyber/common/log.h"
#include "gflags/gflags.h"

#include "modules/common/util/file.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/string_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(resource_monitor_name, "ResourceMonitor",
              "Name of the resource monitor.");

DEFINE_double(resource_monitor_interval, 5,
              "Topic status checking interval (s).");

namespace apollo {
namespace monitor {

using apollo::common::util::StrCat;

ResourceMonitor::ResourceMonitor()
    : RecurrentRunner(FLAGS_resource_monitor_name,
                      FLAGS_resource_monitor_interval) {
}

void ResourceMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  const auto& mode = manager->GetHMIMode();
  auto* components = manager->GetStatus()->mutable_components();
  for (const auto& iter : mode.monitored_components()) {
    const std::string& name = iter.first;
    const auto& config = iter.second;
    if (config.has_resource()) {
      UpdateStatus(config.resource(),
                   components->at(name).mutable_resource_status());
    }
  }
}

void ResourceMonitor::UpdateStatus(
    const apollo::dreamview::ResourceMonitorConfig& config,
    ComponentStatus* status) {
  status->clear_status();
  // Monitor available disk space.
  for (const auto& disk_space : config.disk_spaces()) {
    for (const auto& path : apollo::common::util::Glob(disk_space.path())) {
      const auto space = boost::filesystem::space(path);
      const int available_gb = static_cast<int>(space.available >> 30);
      if (available_gb < disk_space.insufficient_space_error()) {
        const std::string err = StrCat(
            path, " has insufficient space: ",
            available_gb, "GB < ", disk_space.insufficient_space_error());
        SummaryMonitor::EscalateStatus(ComponentStatus::ERROR, err, status);
      } else if (available_gb < disk_space.insufficient_space_warning()) {
        const std::string err = StrCat(
            path, " has insufficient space: ",
            available_gb, "GB < ", disk_space.insufficient_space_warning());
        SummaryMonitor::EscalateStatus(ComponentStatus::WARN, err, status);
      }
    }
  }
  SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", status);
}

}  // namespace monitor
}  // namespace apollo
