/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/software/latency_monitor.h"

#include <algorithm>
#include <memory>

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(latency_monitor_name, "LatencyMonitor",
              "Name of the latency monitor.");

DEFINE_double(latency_monitor_interval, 2.0,
              "Latency report interval in seconds.");

DEFINE_double(latency_report_interval, 10.0,
              "Latency report interval in seconds.");

namespace apollo {
namespace monitor {

using apollo::common::LatencyRecordMap;
using apollo::common::LatencyReport;
using apollo::common::LatencyStat;
using apollo::common::LatencyTrack;

LatencyMonitor::LatencyMonitor()
    : RecurrentRunner(FLAGS_latency_monitor_name,
                      FLAGS_latency_monitor_interval) {}

void LatencyMonitor::RunOnce(const double current_time) {
  static auto reader =
      MonitorManager::Instance()->CreateReader<LatencyRecordMap>(
          FLAGS_latency_recording_topic);
  reader->Observe();
  const auto records = reader->GetLatestObserved();
  if (records == nullptr || records->latency_records().empty()) {
    return;
  }

  UpdateLatencyStat(records);

  if (current_time - flush_time_ > FLAGS_latency_report_interval) {
    flush_time_ = current_time;
    PublishLatencyReport();
  }
}

void LatencyMonitor::UpdateLatencyStat(
    const std::shared_ptr<LatencyRecordMap>& records) {
  if (latency_report_.stat_aggr().find(records->module_name()) ==
      latency_report_.stat_aggr().end()) {
    LatencyStat stat;
    latency_report_.mutable_stat_aggr()->insert({records->module_name(), stat});
  }
  auto* stat = &latency_report_.mutable_stat_aggr()->at(records->module_name());

  uint64_t min_duration = (1UL << 63), max_duration = 0,
           total_duration = 0;
  for (const auto record : records->latency_records()) {
    const auto duration = record.second.end_time() - record.second.begin_time();
    min_duration = std::min(min_duration, duration);
    max_duration = std::max(max_duration, duration);
    total_duration += duration;
    // TODO(Longtao): Track latency accross modules later
    // by using track_map_
    track_map_[record.first].emplace_back(records->module_name(),
                                          record.second.begin_time(),
                                          record.second.end_time());
  }
  stat->set_min_duration(std::min(stat->min_duration(), min_duration));
  stat->set_max_duration(std::max(stat->max_duration(), max_duration));
  const uint32_t sample_size =
      stat->sample_size() +
      static_cast<uint32_t>(records->latency_records().size());
  total_duration += stat->aver_duration() * stat->sample_size();
  stat->set_aver_duration(static_cast<uint64_t>(total_duration / sample_size));
  stat->set_sample_size(sample_size);
}

void LatencyMonitor::PublishLatencyReport() {
  static auto writer = MonitorManager::Instance()->CreateWriter<LatencyReport>(
      FLAGS_latency_reporting_topic);
  apollo::common::util::FillHeader("LatencyReport", &latency_report_);
  writer->Write(latency_report_);
  latency_report_.mutable_stat_aggr()->clear();
}

}  // namespace monitor
}  // namespace apollo
