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
#include "modules/statistics/resource/disk_resource_statistic.h"

#include <regex>
#include <vector>

#include <boost/filesystem.hpp>

#include "absl/strings/str_split.h"

#include "modules/common/util/message_util.h"
#include "modules/statistics/resource/util.h"

namespace apollo {
namespace resource_statistic {
void DiskResourceStatistic::LoadDiskConf(const ResourceStatisticConf_DiskConf& disk_conf) {
    if (disk_conf.device().size() == 0) {
        return;
    }
    // get all load regexes
    std::vector<std::regex> regex_list;
    static const std::string device_prefix = "/dev/";
    for (auto& pattern : disk_conf.device()) {
        if (pattern.empty()) {
            continue;
        }
        auto pattern_ = pattern;
        if (pattern[0] != '/') {
            // add default /dev/
            pattern_ = device_prefix + pattern;
        }
        try {
            regex_list.emplace_back(std::regex(pattern_));
        } catch (std::regex_error& e) {
            AERROR << "error disk conf: " << pattern << ", regex_error caught: " << e.what();
        }
    }

    // find all avaliable disks
    static const std::string mounts_file = "/proc/mounts";
    static constexpr size_t kMaxMounts = 500, kDevice = 0, kPath = 1, kSize = 6;
    const auto mount_lines = GetStatsLines(mounts_file, kMaxMounts);
    for (const auto& line : mount_lines) {
        const std::vector<std::string> mounts = absl::StrSplit(line, ' ', absl::SkipWhitespace());
        // must starts with /dev/
        if (mounts.size() < kSize || mounts[kDevice].substr(0, device_prefix.size()) != device_prefix) {
            continue;
        }
        for (auto& regex_ : regex_list) {
            if (std::regex_match(mounts[kDevice], regex_)) {
                auto device = mounts[kDevice].substr(device_prefix.size());
                disk_load_device_set_.insert(device);
                disk_usage_path_map_.insert({device, mounts[kPath]});
            }
        }
    }
}

bool DiskResourceStatistic::GetDiskMetric(DiskMetric* disk_metric) {
    static const std::string disks_stat_file = "/proc/diskstats";
    const int device_index = 2, in_out_ms = 12;
    static constexpr int kDiskInfo = 128;

    auto devices = disk_metric->mutable_devices();
    const auto stat_lines = GetStatsLines(disks_stat_file, kDiskInfo);
    uint64_t disk_stats = 0;
    for (const auto& line : stat_lines) {
        const std::vector<std::string> stats = absl::StrSplit(line, ' ', absl::SkipWhitespace());
        auto& device = stats[device_index];
        if (disk_load_device_set_.find(device) == disk_load_device_set_.end()) {
            continue;
        }
        disk_stats = std::stoll(stats[in_out_ms]);
        if (disk_load_pre_stat_map_.find(device) == disk_load_pre_stat_map_.end()) {
            disk_load_pre_stat_map_[device] = disk_stats;
            continue;
        }

        auto metric = devices->Add();
        metric->set_name(device);
        // add disk load
        metric->set_load(
                100.f
                * (static_cast<float>(disk_stats - disk_load_pre_stat_map_[device]) / static_cast<float>(interval_)));
        disk_load_pre_stat_map_[device] = disk_stats;
        // add disk usage
        const auto space = boost::filesystem::space(disk_usage_path_map_[device]);
        const auto used_gb = static_cast<uint32_t>((space.capacity - space.available) >> 30);
        const auto total_gb = static_cast<uint32_t>(space.capacity >> 30);
        metric->set_used_gb(used_gb);
        metric->set_available_gb(static_cast<uint32_t>(space.available >> 30));
        metric->set_total_gb(total_gb);
        if (total_gb > 0) {
            metric->set_usage(100.f * used_gb / total_gb);
        }
    }
    return true;
}

}  // namespace resource_statistic
}  // namespace apollo
