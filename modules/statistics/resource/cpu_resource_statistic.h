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
#include "modules/statistics_msgs/resource_statistic.pb.h"

/**
 * @namespace apollo::resource_statistic
 * @brief apollo::resource_statistic
 */
namespace apollo {
namespace resource_statistic {

class CpuResourceStatistic {
public:
    /**
     * @brief Get CPU metric and write it into cpu_metric.
     * @param cpu_metric Cpu metric proto message
     * @return If get cpu metric successfully.
     */
    bool GetCpuMetric(CpuMetric* cpu_metric);

private:
    /**
     * @brief Get CPU usage and write it into cpu_metric.
     * @param cpu_metric Cpu metric proto message
     * @return If get usage successfully.
     */
    bool GetUsage(CpuMetric* cpu_metric);

    /**
     * @brief Get CPU temperature and write it into cpu_metric.
     * @param cpu_metric Cpu metric proto message
     * @return If get temperature successfully.
     */
    bool GetTemperature(CpuMetric* cpu_metric);
};

}  // namespace resource_statistic
}  // namespace apollo
