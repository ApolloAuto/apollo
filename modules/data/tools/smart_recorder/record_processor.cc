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

#include "modules/data/tools/smart_recorder/record_processor.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

#include "modules/data/tools/smart_recorder/bumper_crash_trigger.h"
#include "modules/data/tools/smart_recorder/drive_event_trigger.h"
#include "modules/data/tools/smart_recorder/emergency_mode_trigger.h"
#include "modules/data/tools/smart_recorder/hard_brake_trigger.h"
#include "modules/data/tools/smart_recorder/interval_pool.h"
#include "modules/data/tools/smart_recorder/regular_interval_trigger.h"
#include "modules/data/tools/smart_recorder/small_topics_trigger.h"
#include "modules/data/tools/smart_recorder/swerve_trigger.h"

namespace apollo {
namespace data {

using cyber::common::DirectoryExists;
using cyber::common::EnsureDirectory;
using cyber::common::GetFileName;
using cyber::record::RecordWriter;

RecordProcessor::RecordProcessor(const std::string& source_record_dir,
                                 const std::string& restored_output_dir)
    : source_record_dir_(source_record_dir),
      restored_output_dir_(restored_output_dir) {}

bool RecordProcessor::Init(const SmartRecordTrigger& trigger_conf) {
  // Init input/output
  if (!DirectoryExists(source_record_dir_)) {
    AERROR << "source record dir does not exist: " << source_record_dir_;
    return false;
  }
  if (!EnsureDirectory(restored_output_dir_)) {
    AERROR << "unable to open output dir: " << restored_output_dir_;
    return false;
  }
  // Init triggers
  if (!InitTriggers(trigger_conf)) {
    AERROR << "unable to init triggers";
    return false;
  }
  // Init writer
  static constexpr uint64_t kMBToKB = 1024UL;
  writer_.reset(new RecordWriter());
  writer_->SetIntervalOfFileSegmentation(
      trigger_conf.segment_setting().time_segment());
  writer_->SetSizeOfFileSegmentation(
      trigger_conf.segment_setting().size_segment() * kMBToKB);
  const std::string output_file = GetDefaultOutputFile();
  AINFO << "output file path: " << output_file;
  if (!writer_->Open(output_file)) {
    AERROR << "failed to open file for writing: " << output_file;
    return false;
  }
  // Init intervals pool
  IntervalPool::Instance()->Reset();
  IntervalPool::Instance()->SetIntervalEventLogFilePath(
      trigger_conf.trigger_log_file_path(), GetFileName(restored_output_dir_));
  return true;
}

bool RecordProcessor::InitTriggers(const SmartRecordTrigger& trigger_conf) {
  triggers_.push_back(std::unique_ptr<TriggerBase>(new DriveEventTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new EmergencyModeTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new HardBrakeTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new SmallTopicsTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new RegularIntervalTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new SwerveTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new BumperCrashTrigger));
  for (const auto& trigger : triggers_) {
    if (!trigger->Init(trigger_conf)) {
      AERROR << "unable to initiate trigger and collect channels";
      return false;
    }
  }
  return true;
}

bool RecordProcessor::ShouldRestore(
    const cyber::record::RecordMessage& msg) const {
  for (const auto& trigger : triggers_) {
    if (trigger->ShouldRestore(msg)) {
      return true;
    }
  }
  return false;
}

}  // namespace data
}  // namespace apollo
