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

#pragma once

#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>

#include "modules/common/latency_recorder/proto/latency_record.pb.h"
#include "modules/monitor/common/recurrent_runner.h"

namespace apollo {
namespace monitor {

class LatencyMonitor : public RecurrentRunner {
 public:
  LatencyMonitor();
  void RunOnce(const double current_time) override;
  bool GetFrequency(const std::string& channel_name, double* freq);

 private:
  void UpdateStat(
      const std::shared_ptr<apollo::common::LatencyRecordMap>& records);
  void PublishLatencyReport();
  void AggregateLatency();

  apollo::common::LatencyReport latency_report_;
  std::unordered_map<uint64_t,
                     std::set<std::tuple<uint64_t, uint64_t, std::string>>>
      track_map_;
  std::unordered_map<std::string, double> freq_map_;
  double flush_time_ = 0.0;
};

}  // namespace monitor
}  // namespace apollo
