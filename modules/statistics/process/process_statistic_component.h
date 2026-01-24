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

#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/statistics_msgs/process_statistic.pb.h"

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

/**
 * @namespace apollo::process_statistic
 * @brief apollo::process_statistic
 */
namespace apollo {
namespace process_statistic {

const auto mainboard_dag_regex = std::regex("-d ([\\w/\\.-]+)");
const auto mainboard_process_regex = std::regex("-p ([\\w-]+)");
const uint8_t cache_clean_cycles = 60;

struct CacheMetric {
    uint64_t timestamp;
    std::vector<std::string> mainboard_dags;
    std::string mainboard_process;
    std::string general_process;
};

class ProcessStatisticComponent : public apollo::cyber::TimerComponent {
public:
    bool Init() override;
    bool Proc() override;

private:
    // load conf
    void LoadConf(const ProcessStatisticConf& conf);
    std::vector<std::regex> general_process_regex_list_;

    // cache
    bool ParseFromCache(const std::string& cmd_string, std::shared_ptr<ProcessMetrics> metrics);
    void CleanCache();
    std::unordered_map<std::string, std::shared_ptr<CacheMetric>> cache_;
    uint8_t cycles = 0;

    // parse command
    void ParseCmd(
            const std::string& cmd_string,
            std::shared_ptr<ProcessMetrics> metrics,
            std::shared_ptr<CacheMetric> cache_metric);

    std::shared_ptr<cyber::Writer<ProcessMetrics>> metric_writer_;
};

CYBER_REGISTER_COMPONENT(ProcessStatisticComponent)

}  // namespace process_statistic
}  // namespace apollo
