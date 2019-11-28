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

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "boost/filesystem.hpp"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gflags/gflags.h"

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(resource_monitor_name, "ResourceMonitor",
              "Name of the resource monitor.");

DEFINE_double(resource_monitor_interval, 5,
              "Topic status checking interval (s).");

namespace apollo {
namespace monitor {

namespace {

bool GetPIDByCmdLine(const std::string& process_dag_path, int* pid) {
  const std::string system_proc_path = "/proc";
  const std::string proc_cmdline_path = "/cmdline";
  const auto dirs = cyber::common::ListSubPaths(system_proc_path);
  std::string cmd_line;
  for (const auto& dir_name : dirs) {
    if (!std::all_of(dir_name.begin(), dir_name.end(), isdigit)) {
      continue;
    }
    *pid = std::stoi(dir_name);
    std::ifstream cmdline_file(
        absl::StrCat(system_proc_path, "/", dir_name, proc_cmdline_path));
    std::getline(cmdline_file, cmd_line);
    if (absl::StrContains(cmd_line, process_dag_path)) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> GetStatsList(const std::string& stat_file) {
  std::vector<std::string> stats;
  std::ifstream buffer(stat_file);
  std::string line;
  std::getline(buffer, line);
  std::istringstream iss(line);
  std::copy(std::istream_iterator<std::string>(iss),
            std::istream_iterator<std::string>(), std::back_inserter(stats));
  return stats;
}

float GetMemoryUsage(const int pid, const std::string& process_name) {
  const std::string memory_stat_file = absl::StrCat("/proc/", pid, "/statm");
  const uint32_t page_size_kb = (sysconf(_SC_PAGE_SIZE) >> 10);
  const int resident_idx = 1, gb_2_kb = (1 << 20);

  const auto stats = GetStatsList(memory_stat_file);
  if (stats.size() <= resident_idx) {
    AERROR << "failed to get memory info for process " << process_name;
    return 0.0;
  }
  return static_cast<float>(std::stoi(stats[resident_idx]) * page_size_kb) /
         gb_2_kb;
}

float GetCPUUsage(const int pid, const std::string& process_name,
                  std::unordered_map<std::string, uint64_t>* prev_jiffies_map) {
  const std::string cpu_stat_file = absl::StrCat("/proc/", pid, "/stat");
  const int hertz = sysconf(_SC_CLK_TCK);
  const int utime = 13, stime = 14, cutime = 15, cstime = 16;

  const auto stats = GetStatsList(cpu_stat_file);
  if (stats.size() <= cstime) {
    AERROR << "failed to get CPU info for process " << process_name;
    return 0.0;
  }
  const uint64_t jiffies = std::stoi(stats[utime]) + std::stoi(stats[stime]) +
                           std::stoi(stats[cutime]) + std::stoi(stats[cstime]);
  const uint64_t prev_jiffies = (*prev_jiffies_map)[process_name];
  (*prev_jiffies_map)[process_name] = jiffies;
  if (prev_jiffies == 0) {
    return 0.0;
  }
  return 100.0f * (static_cast<float>(jiffies - prev_jiffies) / hertz /
                  FLAGS_resource_monitor_interval);
}

}  // namespace

ResourceMonitor::ResourceMonitor()
    : RecurrentRunner(FLAGS_resource_monitor_name,
                      FLAGS_resource_monitor_interval) {}

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
  CheckDiskSpace(config, status);

  // Check CPU and Memory.
  // Retrieve the PID first to avoid iterating file systems repeatedly
  const auto process_dag_path = config.process_dag_path();
  if (!process_dag_path.empty()) {
    int pid = 0;
    if (GetPIDByCmdLine(process_dag_path, &pid)) {
      CheckCPUUsage(pid, config, status);
      CheckMemoryUsage(pid, config, status);
    }
  }
  SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", status);
}

void ResourceMonitor::CheckDiskSpace(
    const apollo::dreamview::ResourceMonitorConfig& config,
    ComponentStatus* status) {
  // Monitor available disk space.
  for (const auto& disk_space : config.disk_spaces()) {
    for (const auto& path : cyber::common::Glob(disk_space.path())) {
      const auto space = boost::filesystem::space(path);
      const int available_gb = static_cast<int>(space.available >> 30);
      if (available_gb < disk_space.insufficient_space_error()) {
        const std::string err =
            absl::StrCat(path, " has insufficient space: ", available_gb,
                         "GB < ", disk_space.insufficient_space_error());
        SummaryMonitor::EscalateStatus(ComponentStatus::ERROR, err, status);
      } else if (available_gb < disk_space.insufficient_space_warning()) {
        const std::string err =
            absl::StrCat(path, " has insufficient space: ", available_gb,
                         "GB < ", disk_space.insufficient_space_warning());
        SummaryMonitor::EscalateStatus(ComponentStatus::WARN, err, status);
      }
    }
  }
}

void ResourceMonitor::CheckCPUUsage(
    const int pid, const apollo::dreamview::ResourceMonitorConfig& config,
    ComponentStatus* status) {
  const auto process_dag_path = config.process_dag_path();
  static std::unordered_map<std::string, uint64_t> prev_jiffies_map;
  if (prev_jiffies_map.find(process_dag_path) == prev_jiffies_map.end()) {
    prev_jiffies_map[process_dag_path] = 0;
  }
  const auto cpu_usage = GetCPUUsage(pid, process_dag_path, &prev_jiffies_map);
  const auto high_cpu_warning = config.cpu_usage().high_cpu_usage_warning(),
             high_cpu_error = config.cpu_usage().high_cpu_usage_error();
  if (cpu_usage > high_cpu_error) {
    const std::string err =
        absl::StrCat(process_dag_path, " has high cpu usage: ", cpu_usage,
                     "% > ", high_cpu_error, "%");
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR, err, status);
  } else if (cpu_usage > high_cpu_warning) {
    const std::string warn =
        absl::StrCat(process_dag_path, " has high cpu usage: ", cpu_usage,
                     "% > ", high_cpu_warning, "%");
    SummaryMonitor::EscalateStatus(ComponentStatus::WARN, warn, status);
  }
}

void ResourceMonitor::CheckMemoryUsage(
    const int pid, const apollo::dreamview::ResourceMonitorConfig& config,
    ComponentStatus* status) {
  const auto process_dag_path = config.process_dag_path();
  const auto memory_usage = GetMemoryUsage(pid, process_dag_path);
  const auto high_memory_warning =
                 config.memory_usage().high_memory_usage_warning(),
             high_memory_error =
                 config.memory_usage().high_memory_usage_error();
  if (memory_usage > static_cast<float>(high_memory_error)) {
    const std::string err =
        absl::StrCat(process_dag_path, " has high memory usage: ", memory_usage,
                     " > ", high_memory_error);
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR, err, status);
  } else if (memory_usage > static_cast<float>(high_memory_warning)) {
    const std::string warn =
        absl::StrCat(process_dag_path, " has high memory usage: ", memory_usage,
                     " > ", high_memory_warning);
    SummaryMonitor::EscalateStatus(ComponentStatus::WARN, warn, status);
  }
}

}  // namespace monitor
}  // namespace apollo
