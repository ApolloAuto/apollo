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
#include <mutex>
#include <string>
#include <unordered_map>

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/common/latency_recorder/proto/latency_record.pb.h"

namespace apollo {
namespace common {

class LatencyRecorder {
 public:
  explicit LatencyRecorder(const std::string& module_name);

  void AppendLatencyRecord(const uint64_t message_id, const uint64_t begin_time,
                           const uint64_t end_time);

  void AppendLatencyRecord(const uint64_t message_id, const double begin_time,
                           const double end_time);

 private:
  LatencyRecorder() = default;
  std::shared_ptr<apollo::cyber::Writer<LatencyRecordMap>> CreateWriter();
  void PublishLatencyRecords(
      const std::shared_ptr<apollo::cyber::Writer<LatencyRecordMap>>& writer);

  std::string module_name_;
  std::mutex mutex_;
  std::unique_ptr<LatencyRecordMap> records_ = nullptr;
  uint64_t current_timestamp_ = 0;
  const double publish_interval_ = 3.0;
  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
};

}  // namespace common
}  // namespace apollo
