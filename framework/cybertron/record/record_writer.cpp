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
  file_writer_.reset(new RecordFileWriter());
  HeaderBuilder* header_builder = new HeaderBuilder();
  header_ = header_builder->GetHeader();
  delete header_builder;
  if (!file_writer_->Open(path_)) {
    AERROR << "open outfile failed. file: " << path_;
    return false;
  }
  file_writer_->WriteHeader(header_);
  is_opened_ = true;
  return is_opened_;
}

void RecordWriter::Close() {
  if (is_opened_) {
    file_writer_->Close();
    is_opened_ = false;
  }
}

void RecordWriter::SplitOutfile() {
  file_writer_.reset(new RecordFileWriter());
  path_ = file_ + "." + std::to_string(++file_index_);
  segment_raw_size_ = 0;
  segment_begin_time_ = 0;
  file_writer_->Open(path_);
  file_writer_->WriteHeader(header_);
  for (const auto& i : channel_message_number_map_) {
    Channel channel;
    channel.set_name(i.first);
    channel.set_message_type(channel_message_type_map_[i.first]);
    channel.set_proto_desc(channel_proto_desc_map_[i.first]);
    file_writer_->WriteChannel(channel);
  }
  AINFO << "split out new file: " << path_;
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
      AERROR << "write channel fail.";
      return false;
    }
  } else {
    AWARN << "intercept write channel request, duplicate channel: "
          << channel_name;
  }
  return true;
}

bool RecordWriter::WriteMessage(const SingleMessage& message) {
  std::lock_guard<std::mutex> lg(mutex_);

  if (!file_writer_->WriteMessage(message)) {
    AERROR << "write message fail.";
    return false;
  }

  segment_raw_size_ += message.content().size();
  if (segment_begin_time_ == 0) {
    segment_begin_time_ = message.time();
  }
  if ((header_.segment_interval() > 0 &&
       message.time() - segment_begin_time_ > header_.segment_interval()) ||
      (header_.segment_raw_size() > 0 &&
       segment_raw_size_ > header_.segment_raw_size())) {
    file_writer_backup_.swap(file_writer_);
    file_writer_backup_->Close();
    SplitOutfile();
  }
  return true;
}

void RecordWriter::ShowProgress() {
  static int total = 0;
  std::cout << "\r[RUNNING]  Record : "
            << "    total channel num : " << channel_message_number_map_.size()
            << "  total msg num : " << ++total;
  std::cout.flush();
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo
