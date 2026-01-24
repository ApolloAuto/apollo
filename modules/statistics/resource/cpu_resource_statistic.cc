/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/statistics/resource/cpu_resource_statistic.h"

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include "absl/strings/str_split.h"

#include "cyber/cyber.h"
#include "modules/statistics/resource/util.h"

namespace apollo {
namespace resource_statistic {

// cpu
bool CpuResourceStatistic::GetCpuMetric(CpuMetric* cpu_metric) {
    if (!GetUsage(cpu_metric)) {
        return false;
    }
    if (!GetTemperature(cpu_metric)) {
        return false;
    }
    return true;
}

bool CpuResourceStatistic::GetUsage(CpuMetric* cpu_metric) {
    static uint64_t prev_jiffies = 0, prev_work_jiffies = 0;
    static const std::string system_cpu_stat_file = "/proc/stat";
    const int users = 1, system = 3, total = 7;
    static constexpr int kSystemCpuInfo = 0;
    const auto stat_lines = GetStatsLines(system_cpu_stat_file, kSystemCpuInfo + 1);
    if (stat_lines.size() <= kSystemCpuInfo) {
        AERROR << "failed to load contents from " << system_cpu_stat_file;
        return false;
    }
    const std::vector<std::string> jiffies_stats
            = absl::StrSplit(stat_lines[kSystemCpuInfo], ' ', absl::SkipWhitespace());
    if (jiffies_stats.size() <= total) {
        AERROR << "failed to get system CPU info from " << system_cpu_stat_file;
        return false;
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
        cpu_metric->set_usage(0.f);
    } else {
        cpu_metric->set_usage(
                100.f * (static_cast<float>(work_jiffies - tmp_prev_work_jiffies) / (jiffies - tmp_prev_jiffies)));
    }
    return true;
}

bool CpuResourceStatistic::GetTemperature(CpuMetric* cpu_metric) {
    static const std::string cpu_temp_files_path = "/sys/class/hwmon/hwmon*/temp*_input";
    std::vector<std::string> temp_files = cyber::common::Glob(cpu_temp_files_path);
    if (temp_files.empty()) {
        AERROR << "temperature files not found!";
        return true;
    }
    float temperature = 0;
    for (auto& temp_input : temp_files) {
        std::string temp;
        cyber::common::GetContent(temp_input, &temp);
        temperature = std::max(std::stof(temp), temperature);
    }
    temperature /= 1000;
    cpu_metric->set_temperature(temperature);
    return true;
}

}  // namespace resource_statistic
}  // namespace apollo
