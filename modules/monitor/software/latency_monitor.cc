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
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(latency_monitor_name, "LatencyMonitor",
              "Name of the latency monitor.");

DEFINE_double(latency_monitor_interval, 1.5,
              "Latency report interval in seconds.");

DEFINE_double(latency_report_interval, 15.0,
              "Latency report interval in seconds.");

DEFINE_int32(latency_reader_capacity, 30,
             "The max message numbers in latency reader queue.");

namespace apollo {
namespace monitor {

namespace {

using apollo::common::LatencyRecordMap;
using apollo::common::LatencyReport;
using apollo::common::LatencyStat;
using apollo::common::LatencyTrack;

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
  stat.set_aver_duration(
      sample_size == 0 ? 0 : static_cast<uint64_t>(sum / sample_size));
  stat.set_sample_size(sample_size);
  return stat;
}

void SetStat(const LatencyStat& src, LatencyStat* dst) {
  dst->set_min_duration(src.min_duration());
  dst->set_max_duration(src.max_duration());
  dst->set_aver_duration(src.aver_duration());
  dst->set_sample_size(src.sample_size());
}

void SetLatency(const std::string& latency_name,
                const std::vector<uint64_t>& latency_values,
                LatencyTrack* track) {
  auto* latency_track = track->add_latency_track();
  latency_track->set_latency_name(latency_name);
  SetStat(GenerateStat(latency_values), latency_track->mutable_latency_stat());
}

}  // namespace

LatencyMonitor::LatencyMonitor()
    : RecurrentRunner(FLAGS_latency_monitor_name,
                      FLAGS_latency_monitor_interval) {}

void LatencyMonitor::RunOnce(const double current_time) {
  static auto reader =
      MonitorManager::Instance()->CreateReader<LatencyRecordMap>(
          FLAGS_latency_recording_topic);
  reader->SetHistoryDepth(FLAGS_latency_reader_capacity);
  reader->Observe();

  static std::string last_processed_key;
  std::string first_key_of_current_round;
  for (auto it = reader->Begin(); it != reader->End(); ++it) {
    const std::string current_key =
        absl::StrCat((*it)->module_name(), (*it)->header().sequence_num());
    if (it == reader->Begin()) {
      first_key_of_current_round = current_key;
    }
    if (current_key == last_processed_key) {
      break;
    }
    UpdateStat(*it);
  }
  last_processed_key = first_key_of_current_round;

  if (current_time - flush_time_ > FLAGS_latency_report_interval) {
    flush_time_ = current_time;
    if (!track_map_.empty()) {
      PublishLatencyReport();
    }
  }
}

void LatencyMonitor::UpdateStat(
    const std::shared_ptr<LatencyRecordMap>& records) {
  const auto module_name = records->module_name();
  for (const auto& record : records->latency_records()) {
    track_map_[record.message_id()].emplace(record.begin_time(),
                                            record.end_time(), module_name);
  }

  if (!records->latency_records().empty()) {
    const auto begin_time = records->latency_records().begin()->begin_time();
    const auto end_time = records->latency_records().rbegin()->end_time();
    if (end_time > begin_time) {
      freq_map_[module_name] =
          records->latency_records().size() /
          apollo::cyber::Time(end_time - begin_time).ToSecond();
    }
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
  latency_report_.clear_modules_latency();
  latency_report_.clear_e2es_latency();
}

void LatencyMonitor::AggregateLatency() {
  static const std::string kE2EStartPoint = FLAGS_pointcloud_topic;
  std::unordered_map<std::string, std::vector<uint64_t>> modules_track;
  std::unordered_map<std::string, std::vector<uint64_t>> e2es_track;
  std::unordered_set<std::string> all_modules;

  // Aggregate modules latencies
  std::string module_name;
  uint64_t begin_time = 0, end_time = 0;
  for (const auto& message : track_map_) {
    auto iter = message.second.begin();
    while (iter != message.second.end()) {
      std::tie(begin_time, end_time, module_name) = *iter;
      modules_track[module_name].push_back(end_time - begin_time);
      all_modules.emplace(module_name);
      ++iter;
    }
  }
  // Aggregate E2E latencies
  std::unordered_map<std::string, uint64_t> e2e_latencies;
  for (const auto& message : track_map_) {
    uint64_t e2e_begin_time = 0;
    auto iter = message.second.begin();
    e2e_latencies.clear();
    while (iter != message.second.end()) {
      std::tie(begin_time, std::ignore, module_name) = *iter;
      if (e2e_begin_time == 0 && module_name == kE2EStartPoint) {
        e2e_begin_time = begin_time;
      } else if (module_name != kE2EStartPoint && e2e_begin_time != 0 &&
                 e2e_latencies.find(module_name) == e2e_latencies.end()) {
        const auto duration = begin_time - e2e_begin_time;
        e2e_latencies[module_name] = duration;
        e2es_track[module_name].push_back(duration);
      }
      ++iter;
    }
  }

  // The results could be in the following fromat:
  // e2e latency:
  // pointcloud -> perception: min(500), max(600), average(550),
  // sample_size(1500) pointcloud -> planning: min(800), max(1000),
  // average(900), sample_size(1500) pointcloud -> control: min(1200),
  // max(1300), average(1250), sample_size(1500)
  // ...
  // modules latency:
  // perception: min(5), max(50), average(30), sample_size(1000)
  // prediction: min(500), max(5000), average(2000), sample_size(800)
  // control: min(500), max(800), average(600), sample_size(800)
  // ...

  auto* modules_latency = latency_report_.mutable_modules_latency();
  for (const auto& module : modules_track) {
    SetLatency(module.first, module.second, modules_latency);
  }
  auto* e2es_latency = latency_report_.mutable_e2es_latency();
  for (const auto& e2e : e2es_track) {
    SetLatency(absl::StrCat(kE2EStartPoint, " -> ", e2e.first), e2e.second,
               e2es_latency);
  }
}

bool LatencyMonitor::GetFrequency(const std::string& channel_name,
                                  double* freq) {
  if (freq_map_.find(channel_name) == freq_map_.end()) {
    return false;
  }
  *freq = freq_map_[channel_name];
  return true;
}

}  // namespace monitor
}  // namespace apollo
