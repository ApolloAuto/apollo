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

#include <boost/filesystem.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "gflags/gflags.h"

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
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

std::vector<std::string> GetStatsLines(const std::string& stat_file,
                                       const int line_count) {
  std::vector<std::string> stats_lines;
  std::ifstream buffer(stat_file);
  for (int line_num = 0; line_num < line_count; ++line_num) {
    std::string line;
    std::getline(buffer, line);
    if (line.empty()) {
      break;
    }
    stats_lines.push_back(line);
  }
  return stats_lines;
}

float GetMemoryUsage(const int pid, const std::string& process_name) {
  const std::string memory_stat_file = absl::StrCat("/proc/", pid, "/statm");
  const uint32_t page_size_kb = (sysconf(_SC_PAGE_SIZE) >> 10);
  const int resident_idx = 1, gb_2_kb = (1 << 20);
  constexpr static int kMemoryInfo = 0;

  const auto stat_lines = GetStatsLines(memory_stat_file, kMemoryInfo + 1);
  if (stat_lines.size() <= kMemoryInfo) {
    AERROR << "failed to load contents from " << memory_stat_file;
    return 0.f;
  }
  const std::vector<std::string> stats =
      absl::StrSplit(stat_lines[kMemoryInfo], ' ', absl::SkipWhitespace());
  if (stats.size() <= resident_idx) {
    AERROR << "failed to get memory info for process " << process_name;
    return 0.f;
  }
  return static_cast<float>(std::stoll(stats[resident_idx]) * page_size_kb) /
         gb_2_kb;
}

float GetCPUUsage(const int pid, const std::string& process_name,
                  std::unordered_map<std::string, uint64_t>* prev_jiffies_map) {
  const std::string cpu_stat_file = absl::StrCat("/proc/", pid, "/stat");
  const int hertz = sysconf(_SC_CLK_TCK);
  const int utime = 13, stime = 14, cutime = 15, cstime = 16;
  constexpr static int kCpuInfo = 0;

  const auto stat_lines = GetStatsLines(cpu_stat_file, kCpuInfo + 1);
  if (stat_lines.size() <= kCpuInfo) {
    AERROR << "failed to load contents from " << cpu_stat_file;
    return 0.f;
  }
  const std::vector<std::string> stats =
      absl::StrSplit(stat_lines[kCpuInfo], ' ', absl::SkipWhitespace());
  if (stats.size() <= cstime) {
    AERROR << "failed to get CPU info for process " << process_name;
    return 0.f;
  }
  const uint64_t jiffies = std::stoll(stats[utime]) + std::stoll(stats[stime]) +
                           std::stoll(stats[cutime]) +
                           std::stoll(stats[cstime]);
  const uint64_t prev_jiffies = (*prev_jiffies_map)[process_name];
  (*prev_jiffies_map)[process_name] = jiffies;
  if (prev_jiffies == 0) {
    return 0.f;
  }
  return 100.f * (static_cast<float>(jiffies - prev_jiffies) / hertz /
                  static_cast<float>(FLAGS_resource_monitor_interval));
}

uint64_t GetSystemMemoryValueFromLine(std::string stat_line) {
  constexpr static int kMemoryValueIdx = 1;
  const std::vector<std::string> stats =
      absl::StrSplit(stat_line, ' ', absl::SkipWhitespace());
  if (stats.size() <= kMemoryValueIdx) {
    AERROR << "failed to parse memory from line " << stat_line;
    return 0;
  }
  return std::stoll(stats[kMemoryValueIdx]);
}

float GetSystemMemoryUsage() {
  const std::string system_mem_stat_file = "/proc/meminfo";
  const int mem_total = 0, mem_free = 1, buffers = 3, cached = 4,
            swap_total = 14, swap_free = 15, slab = 21;
  const auto stat_lines = GetStatsLines(system_mem_stat_file, slab + 1);
  if (stat_lines.size() <= slab) {
    AERROR << "failed to load contents from " << system_mem_stat_file;
    return 0.f;
  }
  const auto total_memory =
      GetSystemMemoryValueFromLine(stat_lines[mem_total]) +
      GetSystemMemoryValueFromLine(stat_lines[swap_total]);
  int64_t used_memory = total_memory;
  for (int cur_line = mem_free; cur_line <= slab; ++cur_line) {
    if (cur_line == mem_free || cur_line == buffers || cur_line == cached ||
        cur_line == swap_free || cur_line == slab) {
      used_memory -= GetSystemMemoryValueFromLine(stat_lines[cur_line]);
    }
  }
  return 100.f * (static_cast<float>(used_memory) / total_memory);
}

float GetSystemCPUUsage() {
  const std::string system_cpu_stat_file = "/proc/stat";
  const int users = 1, system = 3, total = 7;
  constexpr static int kSystemCpuInfo = 0;
  static uint64_t prev_jiffies = 0, prev_work_jiffies = 0;
  const auto stat_lines =
      GetStatsLines(system_cpu_stat_file, kSystemCpuInfo + 1);
  if (stat_lines.size() <= kSystemCpuInfo) {
    AERROR << "failed to load contents from " << system_cpu_stat_file;
    return 0.f;
  }
  const std::vector<std::string> jiffies_stats =
      absl::StrSplit(stat_lines[kSystemCpuInfo], ' ', absl::SkipWhitespace());
  if (jiffies_stats.size() <= total) {
    AERROR << "failed to get system CPU info from " << system_cpu_stat_file;
    return 0.f;
  }
  uint64_t jiffies = 0, work_jiffies = 0;
  for (int cur_stat = users; cur_stat <= total; ++cur_stat) {
    const auto cur_stat_value = std::stoll(jiffies_stats[cur_stat]);
    jiffies += cur_stat_value;
    if (cur_stat <= system) {
      work_jiffies += cur_stat_value;
    }
  }
  const uint64_t tmp_prev_jiffies = prev_jiffies;
  const uint64_t tmp_prev_work_jiffies = prev_work_jiffies;
  prev_jiffies = jiffies;
  prev_work_jiffies = work_jiffies;
  if (tmp_prev_jiffies == 0) {
    return 0.f;
  }
  return 100.f * (static_cast<float>(work_jiffies - tmp_prev_work_jiffies) /
                  (jiffies - tmp_prev_jiffies));
}

float GetSystemDiskload(const std::string& device_name) {
  const std::string disks_stat_file = "/proc/diskstats";
  const int device = 2, in_out_ms = 12;
  const int seconds_to_ms = 1000;
  constexpr static int kDiskInfo = 128;
  static uint64_t prev_disk_stats = 0;

  const auto stat_lines = GetStatsLines(disks_stat_file, kDiskInfo);
  uint64_t disk_stats = 0;
  for (const auto& line : stat_lines) {
    const std::vector<std::string> stats =
        absl::StrSplit(line, ' ', absl::SkipWhitespace());
    if (stats[device] == device_name) {
      disk_stats = std::stoll(stats[in_out_ms]);
      break;
    }
  }
  const uint64_t tmp_prev_disk_stats = prev_disk_stats;
  prev_disk_stats = disk_stats;
  if (tmp_prev_disk_stats == 0) {
    return 0.f;
  }
  return 100.f *
         (static_cast<float>(disk_stats - tmp_prev_disk_stats) /
          static_cast<float>(FLAGS_resource_monitor_interval * seconds_to_ms));
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
  CheckCPUUsage(config, status);
  CheckMemoryUsage(config, status);
  CheckDiskLoads(config, status);
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
    const apollo::dreamview::ResourceMonitorConfig& config,
    ComponentStatus* status) {
  for (const auto& cpu_usage : config.cpu_usages()) {
    const auto process_dag_path = cpu_usage.process_dag_path();
    float cpu_usage_value = 0.f;
    if (process_dag_path.empty()) {
      cpu_usage_value = GetSystemCPUUsage();
    } else {
      int pid = 0;
      if (GetPIDByCmdLine(process_dag_path, &pid)) {
        static std::unordered_map<std::string, uint64_t> prev_jiffies_map;
        if (prev_jiffies_map.find(process_dag_path) == prev_jiffies_map.end()) {
          prev_jiffies_map[process_dag_path] = 0;
        }
        cpu_usage_value = GetCPUUsage(pid, process_dag_path, &prev_jiffies_map);
      }
    }
    const auto high_cpu_warning = cpu_usage.high_cpu_usage_warning();
    const auto high_cpu_error = cpu_usage.high_cpu_usage_error();
    if (cpu_usage_value > high_cpu_error) {
      const std::string err = absl::StrCat(
          process_dag_path, " has high cpu usage: ", cpu_usage_value, "% > ",
          high_cpu_error, "%");
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR, err, status);
    } else if (cpu_usage_value > high_cpu_warning) {
      const std::string warn = absl::StrCat(
          process_dag_path, " has high cpu usage: ", cpu_usage_value, "% > ",
          high_cpu_warning, "%");
      SummaryMonitor::EscalateStatus(ComponentStatus::WARN, warn, status);
    }
  }
}

void ResourceMonitor::CheckMemoryUsage(
    const apollo::dreamview::ResourceMonitorConfig& config,
    ComponentStatus* status) {
  for (const auto& memory_usage : config.memory_usages()) {
    const auto process_dag_path = memory_usage.process_dag_path();
    float memory_usage_value = 0.f;
    if (process_dag_path.empty()) {
      memory_usage_value = GetSystemMemoryUsage();
    } else {
      int pid = 0;
      if (GetPIDByCmdLine(process_dag_path, &pid)) {
        memory_usage_value = GetMemoryUsage(pid, process_dag_path);
      }
    }
    const auto high_memory_warning = memory_usage.high_memory_usage_warning();
    const auto high_memory_error = memory_usage.high_memory_usage_error();
    if (memory_usage_value > static_cast<float>(high_memory_error)) {
      const std::string err = absl::StrCat(
          process_dag_path, " has high memory usage: ", memory_usage_value,
          " > ", high_memory_error);
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR, err, status);
    } else if (memory_usage_value > static_cast<float>(high_memory_warning)) {
      const std::string warn = absl::StrCat(
          process_dag_path, " has high memory usage: ", memory_usage_value,
          " > ", high_memory_warning);
      SummaryMonitor::EscalateStatus(ComponentStatus::WARN, warn, status);
    }
  }
}

void ResourceMonitor::CheckDiskLoads(
    const apollo::dreamview::ResourceMonitorConfig& config,
    ComponentStatus* status) {
  for (const auto& disk_load : config.disk_load_usages()) {
    const auto disk_load_value = GetSystemDiskload(disk_load.device_name());
    const auto high_disk_load_warning = disk_load.high_disk_load_warning();
    const auto high_disk_load_error = disk_load.high_disk_load_error();
    if (disk_load_value > static_cast<float>(high_disk_load_error)) {
      const std::string err = absl::StrCat(
          disk_load.device_name(), " has high disk load: ", disk_load_value,
          " > ", high_disk_load_error);
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR, err, status);
    } else if (disk_load_value > static_cast<float>(high_disk_load_warning)) {
      const std::string warn = absl::StrCat(
          disk_load.device_name(), " has high disk load: ", disk_load_value,
          " > ", high_disk_load_warning);
      SummaryMonitor::EscalateStatus(ComponentStatus::WARN, warn, status);
    }
  }
}

}  // namespace monitor
}  // namespace apollo
