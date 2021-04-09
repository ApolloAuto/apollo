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

#include "boost/filesystem.hpp"
#include "gflags/gflags.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(resource_monitor_name, "ResourceMonitor",
              "Name of the resource monitor.");

DEFINE_double(resource_monitor_interval, 5,
              "Topic status checking interval (s).");

namespace apollo {
namespace monitor {

ResourceMonitor::ResourceMonitor(const ResourceConf& config)
    : RecurrentRunner(FLAGS_resource_monitor_name,
                      FLAGS_resource_monitor_interval)
    , config_(config) {
}

void ResourceMonitor::RunOnce(const double current_time) {
  // Monitor directory available size.
  for (const auto& dir_space : config_.dir_spaces()) {
    const int min_available_gb = dir_space.min_available_gb();
    for (const auto& path : apollo::common::util::Glob(dir_space.path())) {
      const auto space = boost::filesystem::space(path);
      const int available_gb = space.available >> 30;
      if (available_gb < min_available_gb) {
        MonitorManager::LogBuffer().ERROR() <<
            path << " has only " << available_gb << "GB space left, while " <<
            min_available_gb << "GB is requried.";
      }
    }
  }
}

}  // namespace monitor
}  // namespace apollo
