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
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "cyber/cyber.h"
#include "cyber/record/record_reader.h"
#include "cyber/tools/cyber_recorder/recorder.h"
#include "modules/common_msgs/monitor_msgs/smart_recorder_status.pb.h"
#include "modules/data/tools/smart_recorder/proto/smart_recorder_triggers.pb.h"
#include "modules/data/tools/smart_recorder/record_processor.h"

namespace apollo {
namespace data {

/**
 * @class RealtimeRecordProcessor
 * @brief Realtime processor against recorded tasks that are being recorded
 */
class RealtimeRecordProcessor : public RecordProcessor {
 public:
  RealtimeRecordProcessor(const std::string& source_record_dir,
                          const std::string& restored_output_dir);
  bool Init(const SmartRecordTrigger& trigger_conf) override;
  bool Process() override;
  std::string GetDefaultOutputFile() const override {
    return absl::StrCat(restored_output_dir_, "/", default_output_filename_);
  };
  void MonitorStatus();
  virtual ~RealtimeRecordProcessor() = default;

 private:
  bool GetNextValidRecord(std::string* record_path) const;
  void RestoreMessage(const uint64_t message_time);
  void PublishStatus(const RecordingState state,
                     const std::string& message) const;
  void ProcessRestoreRecord(const std::string& record_path);
  double GetDuration(const std::string& record_file);

  std::shared_ptr<cyber::record::Recorder> recorder_ = nullptr;
  std::shared_ptr<cyber::Node> smart_recorder_node_ = nullptr;
  std::shared_ptr<cyber::Writer<SmartRecorderStatus>> recorder_status_writer_ =
      nullptr;
  std::vector<std::string> record_files_;
  std::string default_output_filename_;
  std::string restore_path_;
  uint32_t reused_record_num_ = 0;
  uint64_t restore_reader_time_ = 0;
  double max_backward_time_ = 30.0;
  double min_restore_chunk_ = 5.0;
  bool is_terminating_ = false;
  const int recorder_wait_time_ = 5000;
};

}  // namespace data
}  // namespace apollo
