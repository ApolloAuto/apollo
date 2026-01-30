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
#include "modules/statistics/latency/latency_statistic_component.h"

#include <limits>

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/statistics/latency/latency_statistic_gflags.h"

namespace apollo {
namespace latency_statistic {

// the max sample number for calculating latency
constexpr uint32_t MAX_SAMPLE_SIZE = 50;
// the expired time set to 3 seconds
constexpr uint32_t EXPIRED_NANOSECONDS = 3000000000;
constexpr uint32_t NANO_TO_MILLI = 1000000;

bool LatencyStatisticComponent::Init() {
    // hook: Apollo License Verification: v_apollo_park
    // Init Channel Defines
    compensator_topic = FLAGS_compensator_topic;
    perception_topic = FLAGS_perception_obstacle_topic;
    prediction_topic = FLAGS_prediction_topic;
    planning_topic = FLAGS_planning_trajectory_topic;
    control_topic = FLAGS_control_command_topic;
    // create readers
    CreateReaders();
    // create metric writer
    metric_writer_ = node_->CreateWriter<LatencyMetrics>(FLAGS_latency_statistic_topic);
    return true;
}

bool LatencyStatisticComponent::Proc() {
    auto metrics = std::make_shared<LatencyMetrics>();
    apollo::common::util::FillHeader(node_->Name(), metrics.get());

    Result result;
    // get result from metric data
    std::lock_guard<std::mutex> guard(lock_);
    for (auto it = latency_map_.begin(); it != latency_map_.end();) {
        auto& lidar_timestamp = it->first;
        if (latest_stamp_ - lidar_timestamp > EXPIRED_NANOSECONDS || latency_map_.size() > MAX_SAMPLE_SIZE) {
            it = latency_map_.erase(it);
            continue;
        }
        auto& metric = it->second;
        FillResult(lidar_timestamp, metric, &result);
        ++it;
    }
    // calculate latency
    CalculateLatency(result, metrics);
    // write latency metrics
    metric_writer_->Write(metrics);
    return true;
}

void LatencyStatisticComponent::FillResult(const uint64_t& lidar_timestamp, const Metric& metric, Result* result) {
    AINFO << "metric compensator: " << metric.compensator << ", lidar_timestamp: " << lidar_timestamp;
    if (metric.compensator > 0) {
        result->compensator_chain_list.push_back((metric.compensator - lidar_timestamp) / NANO_TO_MILLI);
        result->compensator_proc_list.push_back((metric.compensator - lidar_timestamp) / NANO_TO_MILLI);
    }
    if (metric.perception > 0) {
        result->perception_chain_list.push_back((metric.perception - lidar_timestamp) / NANO_TO_MILLI);
        if (metric.compensator > 0) {
            result->perception_proc_list.push_back((metric.perception - metric.compensator) / NANO_TO_MILLI);
        }
    }
    if (metric.prediction > 0) {
        result->prediction_chain_list.push_back((metric.prediction - lidar_timestamp) / NANO_TO_MILLI);
        if (metric.perception > 0) {
            result->prediction_proc_list.push_back((metric.prediction - metric.perception) / NANO_TO_MILLI);
        }
    }
    if (metric.planning > 0) {
        result->planning_chain_list.push_back((metric.planning - lidar_timestamp) / NANO_TO_MILLI);
        if (metric.prediction > 0) {
            result->planning_proc_list.push_back((metric.planning - metric.prediction) / NANO_TO_MILLI);
        }
    }
    if (metric.control > 0) {
        result->control_chain_list.push_back((metric.control - lidar_timestamp) / NANO_TO_MILLI);
        if (metric.planning > 0) {
            result->control_proc_list.push_back((metric.control - metric.planning) / NANO_TO_MILLI);
        }
    }
}

void LatencyStatisticComponent::CalculateLatency(const Result& result, std::shared_ptr<LatencyMetrics> metrics) {
    // lidar compensator latency
    auto lidar_compensator = metrics->mutable_lidar_compensator();
    GetLatencyMetric(result.compensator_chain_list, lidar_compensator->mutable_chain_ms());
    GetLatencyMetric(result.compensator_proc_list, lidar_compensator->mutable_proc_ms());
    // perception latency
    auto perception = metrics->mutable_perception();
    GetLatencyMetric(result.perception_chain_list, perception->mutable_chain_ms());
    GetLatencyMetric(result.perception_proc_list, perception->mutable_proc_ms());
    // prediction latency
    auto prediction = metrics->mutable_prediction();
    GetLatencyMetric(result.prediction_chain_list, prediction->mutable_chain_ms());
    GetLatencyMetric(result.prediction_proc_list, prediction->mutable_proc_ms());
    // planning latency
    auto planning = metrics->mutable_planning();
    GetLatencyMetric(result.planning_chain_list, planning->mutable_chain_ms());
    GetLatencyMetric(result.planning_proc_list, planning->mutable_proc_ms());
    // control latency
    auto control = metrics->mutable_control();
    GetLatencyMetric(result.control_chain_list, control->mutable_chain_ms());
    GetLatencyMetric(result.control_proc_list, control->mutable_proc_ms());
}

void LatencyStatisticComponent::GetLatencyMetric(const std::vector<uint64_t>& items, LatencyMetrics_Metric* metric) {
    if (items.empty()) {
        return;
    }
    metric->set_sample_size(items.size());

    uint64_t min = std::numeric_limits<uint64_t>::max();
    uint64_t max = 0;
    uint64_t sum = 0;
    for (auto& item : items) {
        if (item > max)
            max = item;
        if (item < min)
            min = item;
        sum += item;
    }
    uint64_t avg = sum / items.size();
    metric->set_min(min);
    metric->set_max(max);
    metric->set_avg(avg);
}

void LatencyStatisticComponent::CreateReaders() {
    // lidar compensator
    CreateReader<apollo::drivers::PointCloud>(compensator_topic);
    // perception
    CreateReader<apollo::perception::PerceptionObstacles>(perception_topic);
    // prediction
    CreateReader<apollo::prediction::PredictionObstacles>(prediction_topic);
    // planing
    CreateReader<apollo::planning::ADCTrajectory>(planning_topic);
    // control
    CreateReader<apollo::control::ControlCommand>(control_topic);
}

template <class T>
void LatencyStatisticComponent::CreateReader(const std::string& channel_name) {
    node_->CreateReader<T>(channel_name, [this, &channel_name](const std::shared_ptr<T>& msg) {
        if (!msg->has_header() || !msg->header().has_timestamp_sec() || !msg->header().has_lidar_timestamp()
            || msg->header().lidar_timestamp() == 0) {
            return;
        }
        auto lidar_timestamp = msg->header().lidar_timestamp();
        uint64_t timestamp = cyber::Time(msg->header().timestamp_sec()).ToNanosecond();
        std::lock_guard<std::mutex> guard(lock_);
        if (latency_map_.find(lidar_timestamp) == latency_map_.end()) {
            latency_map_.emplace(lidar_timestamp, Metric());
        }
        auto& metric = latency_map_.at(lidar_timestamp);
        if (channel_name == compensator_topic) {
            metric.compensator = timestamp;
        } else if (channel_name == perception_topic) {
            metric.perception = timestamp;
        } else if (channel_name == prediction_topic) {
            metric.prediction = timestamp;
        } else if (channel_name == planning_topic) {
            metric.planning = timestamp;
        } else if (channel_name == control_topic) {
            metric.control = timestamp;
        }
        latest_stamp_ = timestamp;
    });
}

}  // namespace latency_statistic
}  // namespace apollo
