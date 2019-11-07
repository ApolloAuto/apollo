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
#include <utility>
#include <vector>

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

namespace {

using apollo::common::LatencyRecordMap;
using apollo::common::LatencyReport;
using apollo::common::LatencyStat;
using apollo::common::util::StrCat;

void FillInStat(const std::string& module_name, const uint64_t duration,
                std::vector<std::pair<std::string, uint64_t>>* stats,
                uint64_t* total_duration) {
  *total_duration += duration;
  stats->emplace_back(module_name, duration);
}

std::vector<std::pair<std::string, uint64_t>> GetFlowTrackStats(
    std::set<std::tuple<uint64_t, uint64_t, std::string>>* module_durations,
    uint64_t* total_duration) {
  const std::string module_connector = "->";
  std::vector<std::pair<std::string, uint64_t>> stats;

  // Generate a list of <module, duration> pair, for example:
  // <percetpion: 100>, <perception->prediction: 10>, <prediction: 50>, ...
  auto iter = module_durations->begin();
  std::string module_name, prev_name;
  uint64_t begin_time = 0, end_time, prev_end_time = 0;
  while (iter != module_durations->end()) {
    std::tie(begin_time, end_time, module_name) = *iter;
    if (!prev_name.empty()) {
      FillInStat(StrCat(prev_name, module_connector, module_name),
                 begin_time - prev_end_time, &stats, total_duration);
    }
    FillInStat(module_name, end_time - begin_time, &stats, total_duration);
    prev_name = module_name;
    prev_end_time = end_time;
    ++iter;
  }

  return stats;
}

LatencyStat GenerateStat(const std::vector<uint64_t>& numbers) {
  LatencyStat stat;
  uint64_t min_number = (1UL << 63), max_number = 0, sum = 0;
  for (const auto number : numbers) {
    min_number = std::min(min_number, number);
    max_number = std::max(max_number, number);
    sum += number;
  }
  const uint32_t sample_size = static_cast<uint32_t>(numbers.size());
  stat.set_min_duration(min_number);
  stat.set_max_duration(max_number);
  stat.set_aver_duration(static_cast<uint64_t>(sum / sample_size));
  stat.set_sample_size(sample_size);
  return stat;
}

}  // namespace

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
  for (const auto& record : records->latency_records()) {
    track_map_[record.first].emplace(record.second.begin_time(),
                                     record.second.end_time(),
                                     records->module_name());
  }
}

void LatencyMonitor::PublishLatencyReport() {
  static auto writer = MonitorManager::Instance()->CreateWriter<LatencyReport>(
      FLAGS_latency_reporting_topic);
  apollo::common::util::FillHeader("LatencyReport", &latency_report_);
  AggregateLatency();
  writer->Write(latency_report_);
  latency_report_.clear_header();
  track_map_.clear();
  latency_report_.mutable_stat_aggr()->clear();
}

void LatencyMonitor::AggregateLatency() {
  std::unordered_map<std::string, std::vector<uint64_t>> tracks;
  std::vector<uint64_t> totals;

  // Aggregate durations by module names
  for (auto& message : track_map_) {
    uint64_t total_duration = 0;
    const auto stats = GetFlowTrackStats(&message.second, &total_duration);
    std::string module_name;
    uint64_t duration = 0;
    totals.push_back(total_duration);
    for (const auto& module_stat : stats) {
      std::tie(module_name, duration) = module_stat;
      tracks[module_name].push_back(duration);
    }
  }

  // The results could be in the following fromat:
  // total: min(500), max(600), average(550), sample_size(1500)
  // perception: min(5), max(50), average(30), sample_size(1000)
  // perception->prediction: min(0), max(5), average(3), sample_size(1000)
  // prediction: min(50), max(500), average(80), sample_size(800)
  // ...
  const auto total_stat = GenerateStat(totals);
  auto* total_duration_aggr = latency_report_.mutable_total_duration();
  total_duration_aggr->set_min_duration(total_stat.min_duration());
  total_duration_aggr->set_max_duration(total_stat.max_duration());
  total_duration_aggr->set_aver_duration(total_stat.aver_duration());
  total_duration_aggr->set_sample_size(total_stat.sample_size());

  auto* stat_aggr = latency_report_.mutable_stat_aggr();
  for (const auto& track : tracks) {
    stat_aggr->insert({track.first, GenerateStat(track.second)});
  }
}

}  // namespace monitor
}  // namespace apollo
