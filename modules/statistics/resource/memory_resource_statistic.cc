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
#include "modules/statistics/resource/memory_resource_statistic.h"

#include "absl/strings/str_split.h"

#include "modules/statistics/resource/util.h"

namespace apollo {
namespace resource_statistic {

// memory
bool MemoryResourceStatistic::GetMemoryMetric(MemoryMetric* mem_metric) {
    static const std::string system_mem_stat_file = "/proc/meminfo";
    const int mem_total = 0, available = 2, line_count = 3;
    const auto stat_lines = GetStatsLines(system_mem_stat_file, line_count);
    if (stat_lines.size() < line_count) {
        AERROR << "failed to load contents from " << system_mem_stat_file;
        return false;
    }
    const auto total_memory = GetMemoryValueFromLine(stat_lines[mem_total]);
    const auto available_memory = GetMemoryValueFromLine(stat_lines[available]);
    if (available_memory > total_memory) {
        AERROR << "available memory is larger than total memory";
        return false;
    }
    const auto used_memory = total_memory - available_memory;
    mem_metric->set_total_mb(total_memory >> 10);
    mem_metric->set_used_mb(used_memory >> 10);
    mem_metric->set_available_mb((total_memory - used_memory) >> 10);
    mem_metric->set_usage(100.f * (static_cast<float>(used_memory) / total_memory));
    return true;
}

uint64_t MemoryResourceStatistic::GetMemoryValueFromLine(const std::string& stat_line) {
    static constexpr int kMemoryValueIdx = 1;
    const std::vector<std::string> stats = absl::StrSplit(stat_line, ' ', absl::SkipWhitespace());
    if (stats.size() <= kMemoryValueIdx) {
        AERROR << "failed to parse memory from line " << stat_line;
        return 0;
    }
    return std::stoll(stats[kMemoryValueIdx]);
}

}  // namespace resource_statistic
}  // namespace apollo
