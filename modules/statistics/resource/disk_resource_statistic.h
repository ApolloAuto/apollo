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

#pragma once

#include <map>
#include <set>
#include <string>

#include "modules/statistics_msgs/resource_statistic.pb.h"

#include "cyber/cyber.h"
/**
 * @namespace apollo::resource_statistic
 * @brief apollo::resource_statistic
 */
namespace apollo {
namespace resource_statistic {

class DiskResourceStatistic {
public:
    /**
     * @brief Set timer component interval.
     * @param interval The time interval
     */
    void SetInterval(uint32_t interval) {
        interval_ = interval;
    }

    /**
     * @brief Load disk conf.
     * @param disk_conf Disk conf message
     */
    void LoadDiskConf(const ResourceStatisticConf_DiskConf& disk_conf);

    /**
     * @brief Get Disk metric and write it into disk_metric.
     * @param disk_metric Disk metric proto message
     * @return If get metric successfully.
     */
    bool GetDiskMetric(DiskMetric* disk_metric);

private:
    uint32_t interval_;
    std::set<std::string> disk_load_device_set_;
    std::map<std::string, std::string> disk_usage_path_map_;
    std::map<std::string, uint64_t> disk_load_pre_stat_map_;
};

}  // namespace resource_statistic
}  // namespace apollo
