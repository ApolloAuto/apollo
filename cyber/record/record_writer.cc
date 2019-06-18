/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "cyber/record/record_writer.h"

#include <iomanip>
#include <iostream>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace record {

using proto::Channel;
using proto::SingleMessage;

RecordWriter::RecordWriter() { header_ = HeaderBuilder::GetHeader(); }

RecordWriter::RecordWriter(const proto::Header& header) { header_ = header; }

RecordWriter::~RecordWriter() { Close(); }

bool RecordWriter::Open(const std::string& file) {
  file_ = file;
  file_index_ = 0;
  sstream_.str(std::string());
  sstream_.clear();
  sstream_ << "." << std::setw(5) << std::setfill('0') << file_index_++;
  if (header_.segment_interval() > 0 || header_.segment_raw_size() > 0) {
    path_ = file_ + sstream_.str();
  } else {
    path_ = file_;
  }
  file_writer_.reset(new RecordFileWriter());
  if (!file_writer_->Open(path_)) {
    AERROR << "Failed to open output record file: " << path_;
    return false;
  }
  if (!file_writer_->WriteHeader(header_)) {
    AERROR << "Failed to write header: " << path_;
    file_writer_->Close();
    return false;
  }
  is_opened_ = true;
  return is_opened_;
}

void RecordWriter::Close() {
  if (is_opened_) {
    file_writer_->Close();
    is_opened_ = false;
  }
}

bool RecordWriter::SplitOutfile() {
  file_writer_.reset(new RecordFileWriter());
  if (file_index_ > 99999) {
    AWARN << "More than 99999 record files had been recored, will restart "
          << "counting from 0.";
    file_index_ = 0;
  }
  sstream_.str(std::string());
  sstream_.clear();
  sstream_ << "." << std::setw(5) << std::setfill('0') << file_index_++;
  path_ = file_ + sstream_.str();
  segment_raw_size_ = 0;
  segment_begin_time_ = 0;
  if (!file_writer_->Open(path_)) {
    AERROR << "Failed to open record file: " << path_;
    return false;
  }
  if (!file_writer_->WriteHeader(header_)) {
    AERROR << "Failed to write header for record file: " << path_;
    return false;
  }
  for (const auto& i : channel_message_number_map_) {
    Channel channel;
    channel.set_name(i.first);
    channel.set_message_type(channel_message_type_map_[i.first]);
    channel.set_proto_desc(channel_proto_desc_map_[i.first]);
    if (!file_writer_->WriteChannel(channel)) {
      AERROR << "Failed to write channel for record file: " << path_;
      return false;
    }
  }
  return true;
}

bool RecordWriter::WriteChannel(const std::string& channel_name,
                                const std::string& message_type,
                                const std::string& proto_desc) {
  std::lock_guard<std::mutex> lg(mutex_);
  if (IsNewChannel(channel_name)) {
    OnNewChannel(channel_name, message_type, proto_desc);
    Channel channel;
    channel.set_name(channel_name);
    channel.set_message_type(message_type);
    channel.set_proto_desc(proto_desc);
    if (!file_writer_->WriteChannel(channel)) {
      AERROR << "Failed to write channel: " << channel_name;
      return false;
    }
  } else {
    AWARN << "Intercept write channel request, duplicate channel: "
          << channel_name;
  }
  return true;
}

bool RecordWriter::WriteMessage(const SingleMessage& message) {
  std::lock_guard<std::mutex> lg(mutex_);
  OnNewMessage(message.channel_name());
  if (!file_writer_->WriteMessage(message)) {
    AERROR << "Write message is failed.";
    return false;
  }

  segment_raw_size_ += message.content().size();
  if (segment_begin_time_ == 0) {
    segment_begin_time_ = message.time();
  }
  if (segment_begin_time_ > message.time()) {
    segment_begin_time_ = message.time();
  }

  if ((header_.segment_interval() > 0 &&
       message.time() - segment_begin_time_ > header_.segment_interval()) ||
      (header_.segment_raw_size() > 0 &&
       segment_raw_size_ > header_.segment_raw_size())) {
    file_writer_backup_.swap(file_writer_);
    file_writer_backup_->Close();
    if (!SplitOutfile()) {
      AERROR << "Split out file is failed.";
      return false;
    }
  }
  return true;
}

bool RecordWriter::SetSizeOfFileSegmentation(uint64_t size_kilobytes) {
  if (is_opened_) {
    AWARN << "Please call this interface before opening file.";
    return false;
  }
  header_.set_segment_raw_size(size_kilobytes * 1024UL);
  return true;
}

bool RecordWriter::SetIntervalOfFileSegmentation(uint64_t time_sec) {
  if (is_opened_) {
    AWARN << "Please call this interface before opening file.";
    return false;
  }
  header_.set_segment_interval(time_sec * 1000000000UL);
  return true;
}

bool RecordWriter::IsNewChannel(const std::string& channel_name) const {
  return channel_message_number_map_.find(channel_name) ==
         channel_message_number_map_.end();
}

void RecordWriter::OnNewChannel(const std::string& channel_name,
                                const std::string& message_type,
                                const std::string& proto_desc) {
  if (IsNewChannel(channel_name)) {
    channel_message_number_map_[channel_name] = 0;
    channel_message_type_map_[channel_name] = message_type;
    channel_proto_desc_map_[channel_name] = proto_desc;
  }
}

void RecordWriter::OnNewMessage(const std::string& channel_name) {
  if (channel_message_number_map_.find(channel_name) !=
      channel_message_number_map_.end()) {
    channel_message_number_map_[channel_name]++;
  }
}

uint64_t RecordWriter::GetMessageNumber(const std::string& channel_name) const {
  auto search = channel_message_number_map_.find(channel_name);
  if (search != channel_message_number_map_.end()) {
    return search->second;
  }
  return 0;
}

const std::string& RecordWriter::GetMessageType(
    const std::string& channel_name) const {
  auto search = channel_message_type_map_.find(channel_name);
  if (search != channel_message_type_map_.end()) {
    return search->second;
  }
  return null_type_;
}

const std::string& RecordWriter::GetProtoDesc(
    const std::string& channel_name) const {
  auto search = channel_proto_desc_map_.find(channel_name);
  if (search != channel_proto_desc_map_.end()) {
    return search->second;
  }
  return null_type_;
}

std::set<std::string> RecordWriter::GetChannelList() const {
  std::set<std::string> channel_list;
  for (auto& item : channel_message_number_map_) {
    channel_list.insert(item.first);
  }
  return channel_list;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
