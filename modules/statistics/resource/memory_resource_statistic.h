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

#include <string>
#include <vector>

#include "modules/statistics_msgs/resource_statistic.pb.h"

#include "cyber/cyber.h"
/**
 * @namespace apollo::resource_statistic
 * @brief apollo::resource_statistic
 */
namespace apollo {
namespace resource_statistic {

class MemoryResourceStatistic {
public:
    /**
     * @brief Get memory metric and write it into mem_metric.
     * @param mem_metric Memory metric message
     * @return If get metric successfully.
     */
    bool GetMemoryMetric(MemoryMetric* mem_metric);

private:
    /**
     * @brief Get memory value from line.
     * @param stat_line The string of line
     * @return The memory value.
     */
    uint64_t GetMemoryValueFromLine(const std::string& stat_line);
};

}  // namespace resource_statistic
}  // namespace apollo
