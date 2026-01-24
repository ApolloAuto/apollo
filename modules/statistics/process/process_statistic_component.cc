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
#include "modules/statistics/process/process_statistic_component.h"

#include <utility>

#include "modules/common/util/message_util.h"
#include "modules/statistics/process/process_statistic_gflags.h"

namespace apollo {
namespace process_statistic {

bool ProcessStatisticComponent::Init() {
    // hook: Apollo License Verification: v_apollo_park
    ProcessStatisticConf conf;
    if (!ComponentBase::GetProtoConfig(&conf)) {
        AERROR << "Unable to load process statistic conf file: " << ComponentBase::ConfigFilePath();
        return false;
    }
    AINFO << "read conf success.";
    LoadConf(conf);
    // create metric writer
    metric_writer_ = node_->CreateWriter<ProcessMetrics>(FLAGS_process_statistic_topic);
    return true;
}

void ProcessStatisticComponent::LoadConf(const ProcessStatisticConf& conf) {
    if (conf.has_general_process()) {
        for (auto& regex_str : conf.general_process().regex()) {
            try {
                general_process_regex_list_.emplace_back(regex_str);
            } catch (std::regex_error& e) {
                AERROR << "error general process conf: " << regex_str << ", regex_error caught: " << e.what();
            }
        }
    }
}

bool ProcessStatisticComponent::Proc() {
    auto metrics = std::make_shared<ProcessMetrics>();
    apollo::common::util::FillHeader(node_->Name(), metrics.get());

    for (const auto& cmd_file : cyber::common::Glob("/proc/[0-9]*/cmdline")) {
        // Get process command string.
        std::string cmd_string;
        if (cyber::common::GetContent(cmd_file, &cmd_string) && !cmd_string.empty()) {
            // In /proc/<PID>/cmdline, the parts are separated with \0, which will be
            // converted back to whitespaces here.
            std::replace(cmd_string.begin(), cmd_string.end(), '\0', ' ');

            if (ParseFromCache(cmd_string, metrics)) {
                continue;
            }
            ParseCmd(cmd_string, metrics, cache_.at(cmd_string));
        }
    }
    // write metrics
    metric_writer_->Write(metrics);

    // clean cache
    if (++cycles % cache_clean_cycles == 0) {
        cycles = 0;
        CleanCache();
    }
    return true;
}

bool ProcessStatisticComponent::ParseFromCache(const std::string& cmd_string, std::shared_ptr<ProcessMetrics> metrics) {
    auto now = cyber::Time().Now().ToNanosecond();
    if (cache_.find(cmd_string) == cache_.end()) {
        std::shared_ptr<CacheMetric> cache_metric = std::make_shared<CacheMetric>();
        cache_metric->timestamp = now;
        cache_[cmd_string] = std::move(cache_metric);
        return false;
    }
    auto& cache_metric = cache_[cmd_string];
    cache_metric->timestamp = now;
    for (const auto& dag : cache_metric->mainboard_dags) {
        metrics->mutable_mainboard_dag()->add_name(dag);
    }
    if (!cache_metric->mainboard_process.empty()) {
        metrics->mutable_mainboard_process_group()->add_name(cache_metric->mainboard_process);
    }
    if (!cache_metric->general_process.empty()) {
        metrics->mutable_general_process()->add_name(cache_metric->general_process);
    }
    return true;
}

void ProcessStatisticComponent::ParseCmd(
        const std::string& cmd_string,
        std::shared_ptr<ProcessMetrics> metrics,
        std::shared_ptr<CacheMetric> cache_metric) {
    // mainboard match
    if (cmd_string.find("mainboard") != cmd_string.npos) {
        auto end = std::sregex_iterator();
        // dag
        auto dag_begin = std::sregex_iterator(cmd_string.begin(), cmd_string.end(), mainboard_dag_regex);

        for (auto it = dag_begin; it != end; ++it) {
            std::string name = (*it).str(1);
            metrics->mutable_mainboard_dag()->add_name(name);
            cache_metric->mainboard_dags.push_back(std::move(name));
        }
        // process group
        auto process_begin = std::sregex_iterator(cmd_string.begin(), cmd_string.end(), mainboard_process_regex);
        for (auto it = process_begin; it != end; ++it) {
            std::string name = (*it).str(1);
            metrics->mutable_mainboard_process_group()->add_name(name);
            cache_metric->mainboard_process = std::move(name);
        }
    } else {
        // general process match
        for (auto& re : general_process_regex_list_) {
            if (std::regex_match(cmd_string, re)) {
                metrics->mutable_general_process()->add_name(cmd_string);
                cache_metric->general_process = cmd_string;
            }
        }
    }
}

void ProcessStatisticComponent::CleanCache() {
    for (auto it = cache_.begin(); it != cache_.end();) {
        // clean process 10 seconds ago
        auto threshold = cyber::Time().Now().ToNanosecond() - 1e+10;
        if (it->second->timestamp < threshold) {
            it = cache_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace process_statistic
}  // namespace apollo
