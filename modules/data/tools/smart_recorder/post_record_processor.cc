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

#include "modules/data/tools/smart_recorder/post_record_processor.h"

#include <dirent.h>

#include <algorithm>
#include <memory>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_viewer.h"

#include "modules/data/tools/smart_recorder/channel_pool.h"
#include "modules/data/tools/smart_recorder/interval_pool.h"

namespace apollo {
namespace data {

using cyber::common::DirectoryExists;
using cyber::record::RecordReader;
using cyber::record::RecordViewer;
using cyber::record::RecordWriter;

bool PostRecordProcessor::Init(const SmartRecordTrigger& trigger_conf) {
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
  if (!RecordProcessor::Init(trigger_conf)) {
    AERROR << "base init failed";
    return false;
  }
  return true;
}

bool PostRecordProcessor::Process() {
  // First scan, get intervals
  for (const std::string& record : source_record_files_) {
    const auto reader = std::make_shared<RecordReader>(
        absl::StrCat(source_record_dir_, "/", record));
    RecordViewer viewer(reader, 0, UINT64_MAX,
                        ChannelPool::Instance()->GetAllChannels());
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
    const auto reader = std::make_shared<RecordReader>(
        absl::StrCat(source_record_dir_, "/", record));
    RecordViewer viewer(reader, 0, UINT64_MAX,
                        ChannelPool::Instance()->GetAllChannels());
    for (const auto& msg : viewer) {
      // If the message fall into generated intervals,
      // or required by any triggers, restore it
      if (IntervalPool::Instance()->MessageFallIntoRange(msg.time) ||
          ShouldRestore(msg)) {
        writer_->WriteChannel(msg.channel_name,
                              reader->GetMessageType(msg.channel_name),
                              reader->GetProtoDesc(msg.channel_name));
        writer_->WriteMessage(msg.channel_name, msg.content, msg.time);
      }
    }
  }
  return true;
}

std::string PostRecordProcessor::GetDefaultOutputFile() const {
  std::string src_file_name = source_record_files_.front();
  const std::string record_flag(".record");
  src_file_name.resize(src_file_name.size() - src_file_name.find(record_flag) +
                       record_flag.size() + 1);
  return absl::StrCat(restored_output_dir_, "/", src_file_name);
}

void PostRecordProcessor::LoadSourceRecords() {
  DIR* dirp = opendir(source_record_dir_.c_str());
  if (dirp == nullptr) {
    AERROR << "failed to open source dir: " << source_record_dir_;
    return;
  }
  struct dirent* dp = nullptr;
  while ((dp = readdir(dirp)) != nullptr) {
    const std::string file_name = dp->d_name;
    if (dp->d_type == DT_REG &&
        file_name.find(".record") != std::string::npos) {
      source_record_files_.push_back(file_name);
    }
  }
  closedir(dirp);
  std::sort(source_record_files_.begin(), source_record_files_.end());
}

}  // namespace data
}  // namespace apollo
