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

#include "modules/common/latency_recorder/latency_recorder.h"

#include "cyber/common/global_data.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace common {

LatencyRecorder::LatencyRecorder(const std::string& module_name)
    : module_name_(module_name) {
  records_.reset(new LatencyRecordMap);
}

void LatencyRecorder::AppendLatencyRecord(const uint64_t message_id,
                                          const absl::Time& begin_time,
                                          const absl::Time& end_time) {
  // TODO(michael): ALERT for now for trouble shooting,
  // CHECK_LT(begin_time, end_time) in the future to enforce the validation
  if (begin_time >= end_time) {
    // In Simulation mode, there might be large number of cases where
    // begin_times equal to end_times, reduce the error frequency in this mode
    static const int kErrorReduceBase = 1000;
    if (!cyber::common::GlobalData::Instance()->IsRealityMode()) {
      AERROR_EVERY(kErrorReduceBase)
          << "latency begin_time: " << begin_time
          << " greater than or equal to end_time: " << end_time << ", "
          << kErrorReduceBase << " times";
      return;
    }
    AERROR << "latency begin_time: " << begin_time
           << " greater than or equal to end_time: " << end_time;
    return;
  }

  static auto writer = CreateWriter();
  if (writer == nullptr || message_id == 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto* latency_record = records_->add_latency_records();
  latency_record->set_begin_time(absl::ToUnixNanos(begin_time));
  latency_record->set_end_time(absl::ToUnixNanos(end_time));
  latency_record->set_message_id(message_id);

  const auto now = absl::Now();
  static const absl::Duration kPublishInterval = absl::Seconds(3);
  if (now - current_time_ > kPublishInterval) {
    PublishLatencyRecords(writer);
    current_time_ = now;
  }
}

std::shared_ptr<apollo::cyber::Writer<LatencyRecordMap>>
LatencyRecorder::CreateWriter() {
  const std::string node_name_prefix = "latency_recorder";
  if (module_name_.empty()) {
    AERROR << "missing module name for sending latency records";
    return nullptr;
  }
  if (node_ == nullptr) {
    current_time_ = absl::Now();
    node_ = apollo::cyber::CreateNode(absl::StrCat(
        node_name_prefix, module_name_, absl::ToUnixNanos(current_time_)));
    if (node_ == nullptr) {
      AERROR << "unable to create node for latency recording";
      return nullptr;
    }
  }
  return node_->CreateWriter<LatencyRecordMap>(FLAGS_latency_recording_topic);
}

void LatencyRecorder::PublishLatencyRecords(
    const std::shared_ptr<apollo::cyber::Writer<LatencyRecordMap>>& writer) {
  records_->set_module_name(module_name_);
  apollo::common::util::FillHeader("LatencyRecorderMap", records_.get());
  writer->Write(*records_);
  records_.reset(new LatencyRecordMap);
}

}  // namespace common
}  // namespace apollo
