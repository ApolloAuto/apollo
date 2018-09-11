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

#ifndef CYBERTRON_RECORD_RECORD_WRITER_H_
#define CYBERTRON_RECORD_RECORD_WRITER_H_

#include <algorithm>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "cybertron/common/log.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/proto/record.pb.h"
#include "cybertron/record/header_builder.h"
#include "cybertron/record/record_file.h"
#include "cybertron/time/time.h"

using ::apollo::cybertron::Time;
using ::apollo::cybertron::message::RawMessage;
using ::apollo::cybertron::record::RecordFileWriter;

namespace apollo {
namespace cybertron {
namespace record {

class RecordWriter {
 public:
  RecordWriter();
  virtual ~RecordWriter();
  bool Open(const std::string& filename);
  void Close();
  bool WriteChannel(const std::string& name, const std::string& type,
                    const std::string& proto_desc);
  bool WriteMessage(const std::string& channel_name,
                    const std::shared_ptr<const RawMessage>& message);
  bool WriteMessage(const SingleMessage& single_msg);
  void ShowProgress();

 private:
  void SplitOutfile();

  bool is_writing_ = false;
  std::string file_ = "";
  std::string path_ = "";
  uint64_t segment_raw_size_ = 0;
  uint64_t segment_begin_time_ = 0;
  uint64_t file_index_ = 0;
  std::mutex mutex_;
  std::vector<std::string> channel_vec_;
  std::unordered_map<std::string, std::string> channel_message_type_map_;
  std::unordered_map<std::string, std::string> channel_proto_desc_map_;
  std::unique_ptr<RecordFileWriter> file_writer_ = nullptr;
  std::unique_ptr<RecordFileWriter> file_writer_backup_ = nullptr;
  Header header_;
  std::mutex flush_mutex_;
};

inline bool RecordWriter::WriteMessage(
    const std::string& channel_name,
    const std::shared_ptr<const RawMessage>& message) {
  if (message == nullptr) {
    AERROR << "nullptr error, channel: " << channel_name;
    return false;
  }
  SingleMessage single_msg;
  single_msg.set_channel_name(channel_name);
  single_msg.set_content(message->message);
  single_msg.set_time(Time::Now().ToNanosecond());
  if (!WriteMessage(std::move(single_msg))) {
    AERROR << "write single msg fail";
    return false;
  }
  return true;
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_RECORD_WRITER_H_
