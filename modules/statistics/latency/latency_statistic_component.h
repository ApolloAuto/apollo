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
#include <memory>
#include <regex>
#include <string>
#include <vector>

#include "modules/statistics_msgs/latency_statistic.pb.h"

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

/**
 * @namespace apollo::latency_statistic
 * @brief apollo::latency_statistic
 */
namespace apollo {
namespace latency_statistic {

struct Metric {
    uint64_t compensator = 0;
    uint64_t perception = 0;
    uint64_t prediction = 0;
    uint64_t planning = 0;
    uint64_t control = 0;
};

struct Result {
    std::vector<uint64_t> compensator_chain_list;
    std::vector<uint64_t> compensator_proc_list;
    std::vector<uint64_t> perception_chain_list;
    std::vector<uint64_t> perception_proc_list;
    std::vector<uint64_t> prediction_chain_list;
    std::vector<uint64_t> prediction_proc_list;
    std::vector<uint64_t> planning_chain_list;
    std::vector<uint64_t> planning_proc_list;
    std::vector<uint64_t> control_chain_list;
    std::vector<uint64_t> control_proc_list;
};

class LatencyStatisticComponent : public apollo::cyber::TimerComponent {
public:
    bool Init() override;
    bool Proc() override;

private:
    void CreateReaders();

    template <class T>
    void CreateReader(const std::string& channel_name);
    void FillResult(const uint64_t& lidar_timestamp, const Metric& metric, Result* result);
    void CalculateLatency(const Result& result, std::shared_ptr<LatencyMetrics> metrics);

    void GetLatencyMetric(const std::vector<uint64_t>& items, LatencyMetrics_Metric* metric);

    std::map<uint64_t, Metric> latency_map_;
    uint64_t latest_stamp_ = 0;
    std::mutex lock_;

    std::shared_ptr<cyber::Writer<LatencyMetrics>> metric_writer_;

    std::string compensator_topic;
    std::string perception_topic;
    std::string prediction_topic;
    std::string planning_topic;
    std::string control_topic;
};

CYBER_REGISTER_COMPONENT(LatencyStatisticComponent)

}  // namespace latency_statistic
}  // namespace apollo
