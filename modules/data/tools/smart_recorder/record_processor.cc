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

#include <dirent.h>

#include <algorithm>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_viewer.h"
#include "modules/common/util/string_util.h"

#include "modules/data/tools/smart_recorder/drive_event_trigger.h"
#include "modules/data/tools/smart_recorder/emergency_mode_trigger.h"
#include "modules/data/tools/smart_recorder/interval_pool.h"
#include "modules/data/tools/smart_recorder/small_topics_trigger.h"

namespace apollo {
namespace data {

using apollo::common::util::StrCat;
using cyber::common::DirectoryExists;
using cyber::common::EnsureDirectory;
using cyber::common::GetProtoFromFile;
using cyber::message::ProtobufFactory;
using cyber::record::RecordMessage;
using cyber::record::RecordReader;
using cyber::record::RecordViewer;
using cyber::record::RecordWriter;

RecordProcessor::RecordProcessor(std::string source_record_dir,
                                 std::string restored_output_dir)
    : source_record_dir_(source_record_dir),
      restored_output_dir_(restored_output_dir) {}

bool RecordProcessor::Init(const SmartRecordTrigger& trigger_conf) {
  // Init input/output
  if (!DirectoryExists(source_record_dir_)) {
    AERROR << "source record dir does not exist: " << source_record_dir_;
    return false;
  }
  LoadSourceRecords();
  if (source_record_files_.empty()) {
    AERROR << "source record dir does not have any records: "
           << source_record_dir_;
    return false;
  }
  EnsureDirectory(restored_output_dir_);
  // Init triggers, along with their wanted channels
  triggers_.push_back(std::unique_ptr<TriggerBase>(new DriveEventTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new EmergencyModeTrigger));
  triggers_.push_back(std::unique_ptr<TriggerBase>(new SmallTopicsTrigger));
  CollectAllChannels(trigger_conf);
  if (channels_types_map_.empty() || all_channels_.empty()) {
    AERROR << "no channels are defined in triggers";
    return false;
  }
  // Init writer
  writer_.reset(new RecordWriter());
  writer_->SetIntervalOfFileSegmentation(
      trigger_conf.segment_setting().size_segment() * 1024UL);
  writer_->SetSizeOfFileSegmentation(
      trigger_conf.segment_setting().time_segment() * 1000000000UL);
  const std::string output_file = GetDefaultOutputFile();
  AINFO << "output file path: " << output_file;
  if (!writer_->Open(output_file)) {
    AERROR << "failed to open file for writing: " << output_file;
    return false;
  }
  // Init intervals pool
  IntervalPool::Instance()->Reset();
  return true;
}

bool RecordProcessor::Process() {
  // First scan, get intervals
  for (const std::string& record : source_record_files_) {
    const auto reader =
        std::make_shared<RecordReader>(StrCat(source_record_dir_, "/", record));
    RecordViewer viewer(reader, 0, UINT64_MAX, all_channels_);
    AINFO << record << ":" << viewer.begin_time() << " - " << viewer.end_time();
    for (const auto& msg : viewer) {
      for (const auto& trigger : triggers_) {
        trigger->Pull(msg);
      }
    }
  }
  // Second scan, restore messages based on intervals in the first scan
  IntervalPool::Instance()->ReorgIntervals();
  IntervalPool::Instance()->PrintIntervals();
  for (const std::string& record : source_record_files_) {
    const auto reader =
        std::make_shared<RecordReader>(StrCat(source_record_dir_, "/", record));
    RecordViewer viewer(reader, 0, UINT64_MAX, all_channels_);
    for (const auto& msg : viewer) {
      // If the message fall into generated intervals,
      // or required by any triggers, restore it
      if (IntervalPool::Instance()->MessageFallIntoRange(msg.channel_name,
                                                         msg.time) ||
          ShouldRestore(msg)) {
        RestoreMessage(msg);
      }
    }
  }
  return true;
}

void RecordProcessor::LoadSourceRecords() {
  DIR* dirp = opendir(source_record_dir_.c_str());
  struct dirent* dp = nullptr;
  while ((dp = readdir(dirp)) != nullptr) {
    const std::string file_name = dp->d_name;
    if (dp->d_type == DT_REG &&
        file_name.find(".record") != std::string::npos) {
      source_record_files_.push_back(file_name);
    }
  }
  std::sort(source_record_files_.begin(), source_record_files_.end());
}

void RecordProcessor::CollectAllChannels(
    const SmartRecordTrigger& trigger_conf) {
  // Collect all channels from config file, for reading records effectively
  for (const auto& trigger : triggers_) {
    if (!trigger->Init(trigger_conf)) {
      AERROR << "unable to initiate trigger and collect channels";
      return;
    }
    const auto channels_types = trigger->GetChannelTypes();
    channels_types_map_.insert(channels_types.begin(), channels_types.end());
  }
  for (const auto& iter : channels_types_map_) {
    all_channels_.insert(iter.first);
  }
}

void RecordProcessor::RestoreMessage(const RecordMessage& msg) {
  auto* raw_message = ProtobufFactory::Instance()->GenerateMessageByType(
      channels_types_map_[msg.channel_name]);
  CHECK(raw_message) << "Failed to do reflection from " << msg.channel_name;
  std::string get_desc_str;
  ProtobufFactory::GetDescriptorString(*raw_message, &get_desc_str);
  writer_->WriteChannel(msg.channel_name,
                        raw_message->GetDescriptor()->full_name(),
                        get_desc_str);
  writer_->WriteMessage(msg.channel_name, msg.content, msg.time);
}

std::string RecordProcessor::GetDefaultOutputFile() const {
  std::string src_file_name = source_record_files_.front();
  const std::string record_flag(".record");
  src_file_name.resize(src_file_name.size() - src_file_name.find(record_flag) +
                       record_flag.size() + 1);
  return StrCat(restored_output_dir_, "/", src_file_name);
}

bool RecordProcessor::ShouldRestore(const RecordMessage& msg) const {
  for (const auto& trigger : triggers_) {
    if (trigger->ShouldRestore(msg)) {
      return true;
    }
  }
  return false;
}

}  // namespace data
}  // namespace apollo
