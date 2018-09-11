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

#include "cybertron/record/record_writer.h"

namespace apollo {
namespace cybertron {
namespace record {

RecordWriter::RecordWriter() {}

RecordWriter::~RecordWriter() { Close(); }

bool RecordWriter::Open(const std::string& file) {
  file_ = file;
  path_ = file_;
  segment_raw_size_ = 0;
  segment_begin_time_ = 0;
  file_index_ = 0;
  file_writer_.reset(new RecordFileWriter());
  HeaderBuilder* header_builder = new HeaderBuilder();
  header_ = header_builder->GetHeader();
  delete header_builder;
  if (!file_writer_->Open(path_)) {
    AERROR << "open outfile failed. file: " << path_;
    return false;
  }
  file_writer_->WriteHeader(header_);
  is_writing_ = true;
  return true;
}

void RecordWriter::Close() {
  if (is_writing_) {
    file_writer_->Close();
    is_writing_ = false;
  }
}

void RecordWriter::SplitOutfile() {
  file_writer_.reset(new RecordFileWriter());
  path_ = file_ + "." + std::to_string(++file_index_);
  segment_raw_size_ = 0;
  segment_begin_time_ = 0;
  file_writer_->Open(path_);
  file_writer_->WriteHeader(header_);
  for (auto channel_name : channel_vec_) {
    Channel channel;
    channel.set_name(channel_name);
    channel.set_message_type(channel_message_type_map_[channel_name]);
    channel.set_proto_desc(channel_proto_desc_map_[channel_name]);
    file_writer_->WriteChannel(channel);
  }
  AINFO << "split out new file: " << path_;
}

bool RecordWriter::WriteChannel(const std::string& channel_name,
                                const std::string& message_type,
                                const std::string& proto_desc) {
  std::lock_guard<std::mutex> lg(mutex_);
  if (std::find(channel_vec_.begin(), channel_vec_.end(), channel_name) !=
      channel_vec_.end()) {
    return true;
  }
  channel_vec_.push_back(channel_name);
  channel_message_type_map_[channel_name] = message_type;
  channel_proto_desc_map_[channel_name] = proto_desc;
  Channel channel;
  channel.set_name(channel_name);
  channel.set_message_type(message_type);
  channel.set_proto_desc(proto_desc);
  if (!file_writer_->WriteChannel(channel)) {
    AERROR << "write channel fail.";
    return false;
  }
  return true;
}

bool RecordWriter::WriteMessage(const SingleMessage& message) {
  std::lock_guard<std::mutex> lg(mutex_);

  segment_raw_size_ += message.content().size();
  if (segment_begin_time_ == 0) {
    segment_begin_time_ = message.time();
  }

  if (!file_writer_->AddSingleMessage(message)) {
    AERROR << "write message fail.";
    return false;
  }

  if (0 == header_.segment_interval() && 0 == header_.segment_raw_size()) {
    return true;
  }

  if (message.time() - segment_begin_time_ < header_.segment_interval() &&
      segment_raw_size_ < header_.segment_raw_size()) {
    return true;
  }

  file_writer_backup_.swap(file_writer_);
  file_writer_backup_->Close();
  SplitOutfile();
  return true;
}

void RecordWriter::ShowProgress() {
  static int total = 0;
  std::cout << "\r[RUNNING]  Record : "
            << "    total channel num : " << channel_vec_.size()
            << "  total msg num : " << ++total;
  std::cout.flush();
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
