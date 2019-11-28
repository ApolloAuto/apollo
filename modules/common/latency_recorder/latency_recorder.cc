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

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace common {

namespace {

using apollo::cyber::Time;

uint64_t ToNanoSeconds(const double seconds) {
  static constexpr double kNanoSecondsFactor = 1e9;
  return static_cast<uint64_t>(kNanoSecondsFactor * seconds);
}

};  // namespace

LatencyRecorder::LatencyRecorder(const std::string& module_name)
    : module_name_(module_name) {
  records_.reset(new LatencyRecordMap);
}

void LatencyRecorder::AppendLatencyRecord(const uint64_t message_id,
                                          const double begin_time,
                                          const double end_time) {
  AppendLatencyRecord(message_id, ToNanoSeconds(begin_time),
                      ToNanoSeconds(end_time));
}

void LatencyRecorder::AppendLatencyRecord(const uint64_t message_id,
                                          const uint64_t begin_time,
                                          const uint64_t end_time) {
  static auto writer = CreateWriter();
  if (writer == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  LatencyRecord record;
  record.set_begin_time(begin_time);
  record.set_end_time(end_time);
  records_->mutable_latency_records()->insert({message_id, record});

  const auto now_nano_time = Time::Now().ToNanosecond();
  if (now_nano_time - current_timestamp_ >
      Time(publish_interval_).ToNanosecond()) {
    PublishLatencyRecords(writer);
    current_timestamp_ = now_nano_time;
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
    current_timestamp_ = apollo::cyber::Time::Now().ToNanosecond();
    node_ = apollo::cyber::CreateNode(
        absl::StrCat(node_name_prefix, module_name_, current_timestamp_));
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
