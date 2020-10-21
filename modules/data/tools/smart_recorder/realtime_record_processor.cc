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

#include "modules/data/tools/smart_recorder/realtime_record_processor.h"

#include <signal.h>

#include <algorithm>
#include <chrono>
#include <set>
#include <sstream>
#include <thread>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/init.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_viewer.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/monitor/common/monitor_manager.h"

#include "modules/data/tools/smart_recorder/channel_pool.h"
#include "modules/data/tools/smart_recorder/interval_pool.h"

namespace apollo {
namespace data {

namespace {

using apollo::common::Header;
using apollo::common::util::StrCat;
using apollo::monitor::MonitorManager;
using cyber::CreateNode;
using cyber::common::EnsureDirectory;
using cyber::common::GetFileName;
using cyber::common::PathExists;
using cyber::common::RemoveAllFiles;
using cyber::record::HeaderBuilder;
using cyber::record::Recorder;
using cyber::record::RecordFileReader;
using cyber::record::RecordMessage;
using cyber::record::RecordReader;
using cyber::record::RecordViewer;

std::string GetNextRecordFileName(const std::string& record_path) {
  constexpr int kSuffixLen = 5;
  const std::string kInitialSequence = "00000";
  if (record_path.empty()) {
    return kInitialSequence;
  }
  std::stringstream record_suffix;
  record_suffix.fill('0');
  record_suffix.width(kSuffixLen);
  record_suffix << std::to_string(
      std::stoi(
          record_path.substr(record_path.size() - kSuffixLen, kSuffixLen)) +
      1);
  return record_suffix.str();
}

bool IsRecordValid(const std::string& record_path) {
  if (!PathExists(record_path)) {
    return false;
  }
  const std::unique_ptr<RecordFileReader> file_reader(new RecordFileReader());
  if (!file_reader->Open(record_path)) {
    AERROR << "failed to open record file for checking header: " << record_path;
    return false;
  }
  const bool is_complete = file_reader->GetHeader().is_complete();
  file_reader->Close();
  return is_complete;
}

}  // namespace

RealtimeRecordProcessor::RealtimeRecordProcessor(
    const std::string& source_record_dir,
    const std::string& restored_output_dir)
    : RecordProcessor(source_record_dir, restored_output_dir) {
  default_output_filename_ = restored_output_dir_;
  default_output_filename_.erase(
      std::remove(default_output_filename_.begin(),
                  default_output_filename_.end(), '-'),
      default_output_filename_.end());
  default_output_filename_ =
      GetFileName(StrCat(default_output_filename_, ".record"), false);
}

bool RealtimeRecordProcessor::Init(const SmartRecordTrigger& trigger_conf) {
  // Init input/output, for realtime processor create both
  // input and output dir if they do not exist
  if (!EnsureDirectory(source_record_dir_) ||
      !EnsureDirectory(restored_output_dir_)) {
    AERROR << "unable to init input/output dir: " << source_record_dir_ << "/"
           << restored_output_dir_;
    return false;
  }
  if (!RemoveAllFiles(source_record_dir_)) {
    AERROR << "unable to clear input dir: " << source_record_dir_;
    return false;
  }
  // Init recorder
  cyber::Init("smart_recorder");
  smart_recorder_node_ =
      CreateNode(StrCat("smart_recorder_", std::to_string(getpid())));
  if (smart_recorder_node_ == nullptr) {
    AERROR << "create smart recorder node failed: " << getpid();
    return false;
  }
  recorder_status_writer_ =
      smart_recorder_node_->CreateWriter<SmartRecorderStatus>(
          FLAGS_recorder_status_topic);
  max_backward_time_ = trigger_conf.max_backward_time();
  min_restore_chunk_ = trigger_conf.min_restore_chunk();
  std::vector<std::string> all_channels;
  std::vector<std::string> black_channels;
  const std::set<std::string>& all_channels_set =
      ChannelPool::Instance()->GetAllChannels();
  std::copy(all_channels_set.begin(), all_channels_set.end(),
            std::back_inserter(all_channels));
  recorder_ = std::make_shared<Recorder>(
      StrCat(source_record_dir_, "/", default_output_filename_), false,
      all_channels, black_channels, HeaderBuilder::GetHeader());
  // Init base
  if (!RecordProcessor::Init(trigger_conf)) {
    AERROR << "base init failed";
    return false;
  }
  return true;
}

bool RealtimeRecordProcessor::Process() {
  // Recorder goes first
  recorder_->Start();
  PublishStatus(RecordingState::RECORDING, "smart recorder started");
  MonitorManager::Instance()->LogBuffer().INFO("SmartRecorder is recording...");
  std::shared_ptr<std::thread> monitor_thread =
      std::make_shared<std::thread>([this]() { this->MonitorStatus(); });
  // Now fast reader follows and reacts for any events
  std::string record_path;
  do {
    if (!GetNextValidRecord(&record_path)) {
      AINFO << "record reader " << record_path << " reached end, exit now";
      break;
    }
    auto reader = std::make_shared<RecordReader>(record_path);
    RecordViewer viewer(reader, 0, UINT64_MAX,
                        ChannelPool::Instance()->GetAllChannels());
    AINFO << "checking " << record_path << ": " << viewer.begin_time() << " - "
          << viewer.end_time();
    if (restore_reader_time_ == 0) {
      restore_reader_time_ = viewer.begin_time();
      GetNextValidRecord(&restore_path_);
    }
    for (const auto& msg : viewer) {
      for (const auto& trigger : triggers_) {
        trigger->Pull(msg);
      }
      // Slow reader restores the events if any
      RestoreMessage(msg.time);
    }
  } while (!is_terminating_);
  // Try restore the rest of messages one last time
  RestoreMessage(UINT64_MAX);
  if (monitor_thread && monitor_thread->joinable()) {
    monitor_thread->join();
    monitor_thread = nullptr;
  }
  PublishStatus(RecordingState::STOPPED, "smart recorder stopped");
  MonitorManager::Instance()->LogBuffer().INFO("SmartRecorder is stopped");
  return true;
}

void RealtimeRecordProcessor::MonitorStatus() {
  int status_counter = 0;
  while (!cyber::IsShutdown()) {
    static constexpr int kCheckingFrequency = 100;
    static constexpr int kPublishStatusFrequency = 30;
    std::this_thread::sleep_for(std::chrono::milliseconds(kCheckingFrequency));
    if (++status_counter % kPublishStatusFrequency == 0) {
      status_counter = 0;
      PublishStatus(RecordingState::RECORDING, "smart recorder recording");
    }
  }
  recorder_->Stop();
  is_terminating_ = true;
  AINFO << "wait for a while trying to complete the restore work";
  constexpr int kMessageInterval = 1000;
  int interval_counter = 0;
  while (++interval_counter * kMessageInterval < recorder_wait_time_) {
    MonitorManager::Instance()->LogBuffer().WARN(
        "SmartRecorder is terminating...");
    std::this_thread::sleep_for(std::chrono::milliseconds(kMessageInterval));
  }
}

void RealtimeRecordProcessor::PublishStatus(const RecordingState state,
                                            const std::string& message) const {
  SmartRecorderStatus status;
  Header* status_headerpb = status.mutable_header();
  status_headerpb->set_timestamp_sec(cyber::Time::Now().ToSecond());
  status.set_recording_state(state);
  status.set_state_message(message);
  AINFO << "send message with state " << state << ", " << message;
  recorder_status_writer_->Write(status);
}

bool RealtimeRecordProcessor::GetNextValidRecord(
    std::string* record_path) const {
  *record_path = StrCat(source_record_dir_, "/", default_output_filename_, ".",
                        GetNextRecordFileName(*record_path));
  while (!is_terminating_ && !IsRecordValid(*record_path)) {
    AINFO << "next record unavailable, wait " << recorder_wait_time_ << " ms";
    std::this_thread::sleep_for(std::chrono::milliseconds(recorder_wait_time_));
  }
  return IsRecordValid(*record_path);
}

void RealtimeRecordProcessor::RestoreMessage(const uint64_t message_time) {
  // Check and restore messages, logic is:
  // 1. If new events got triggered, restore reader proceeds all the way to the
  //    event's end
  // 2. If no events got triggered, but given message leads the restore reader
  //    by more than max value, proceeds to the max value point
  // 3. Otherwise, do nothing
  const struct Interval interval = IntervalPool::Instance()->GetNextInterval();
  const uint64_t target_end = std::max(
      interval.end_time,
      message_time - static_cast<uint64_t>(max_backward_time_ * 1000000000UL));
  const bool small_channels_only = restore_reader_time_ >= interval.end_time;
  if (small_channels_only &&
      target_end <=
          restore_reader_time_ +
              static_cast<uint64_t>(min_restore_chunk_ * 1000000000UL)) {
    return;
  }
  do {
    if (!IsRecordValid(restore_path_)) {
      AWARN << "invalid restore path " << restore_path_ << ", exit";
      break;
    }
    AINFO << "target restoring " << restore_path_ << ": "
          << restore_reader_time_ << " - " << target_end;
    auto reader = std::make_shared<RecordReader>(restore_path_);
    restore_reader_time_ =
        std::max(restore_reader_time_, reader->GetHeader().begin_time());
    if (restore_reader_time_ > target_end ||
        reader->GetHeader().begin_time() >= reader->GetHeader().end_time()) {
      AWARN << "record " << restore_path_ << " begin_time beyond target, exit";
      break;
    }
    RecordViewer viewer(reader, restore_reader_time_, target_end,
                        ChannelPool::Instance()->GetAllChannels());
    AINFO << "actual restoring " << restore_path_ << ": " << viewer.begin_time()
          << " - " << viewer.end_time();
    for (const auto& msg : viewer) {
      if ((!small_channels_only && msg.time >= interval.begin_time &&
           msg.time <= interval.end_time) ||
          ShouldRestore(msg)) {
        if (writer_->IsNewChannel(msg.channel_name)) {
          writer_->WriteChannel(msg.channel_name,
                                reader->GetMessageType(msg.channel_name),
                                reader->GetProtoDesc(msg.channel_name));
        }
        writer_->WriteMessage(msg.channel_name, msg.content, msg.time);
      }
    }
    restore_reader_time_ = std::min(reader->GetHeader().end_time(), target_end);
    if (target_end >= reader->GetHeader().end_time()) {
      GetNextValidRecord(&restore_path_);
    }
  } while (restore_reader_time_ < target_end);
}

}  // namespace data
}  // namespace apollo
