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
#include "cybertron/message/raw_message.h"
#include "cybertron/record/header_builder.h"
#include "cybertron/record/record_base.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace record {

using ::apollo::cybertron::Time;
using ::apollo::cybertron::message::RawMessage;
using ::apollo::cybertron::record::RecordFileWriter;

class RecordWriter : public RecordBase {
 public:
  RecordWriter();

  virtual ~RecordWriter();

  bool Open(const std::string& file);

  void Close();

  bool WriteChannel(const std::string& channel_name,
                    const std::string& message_type,
                    const std::string& proto_desc);

  template <typename MessageT>
  bool WriteMessage(const std::string& channel_name, const MessageT& message,
                    const uint64_t time, const std::string& proto_desc = "");
  void ShowProgress();

 private:
  bool WriteMessage(const SingleMessage& single_msg);
  void SplitOutfile();

  uint64_t segment_raw_size_ = 0;
  uint64_t segment_begin_time_ = 0;
  uint64_t file_index_ = 0;
  std::unique_ptr<RecordFileWriter> file_writer_ = nullptr;
  std::unique_ptr<RecordFileWriter> file_writer_backup_ = nullptr;
};

template <>
inline bool RecordWriter::WriteMessage(const std::string& channel_name,
                                       const std::string& content,
                                       const uint64_t time,
                                       const std::string& proto_desc) {
  OnNewMessage(channel_name);
  SingleMessage single_msg;
  single_msg.set_channel_name(channel_name);
  single_msg.set_content(content);
  single_msg.set_time(time);
  return WriteMessage(single_msg);
}

template <>
inline bool RecordWriter::WriteMessage(
    const std::string& channel_name, const std::shared_ptr<RawMessage>& message,
    const uint64_t time, const std::string& proto_desc) {
  if (message == nullptr) {
    AERROR << "nullptr error, channel: " << channel_name;
    return false;
  }
  return WriteMessage(channel_name, message->message, time);
}

template <typename MessageT>
bool RecordWriter::WriteMessage(const std::string& channel_name,
                                const MessageT& message, const uint64_t time,
                                const std::string& proto_desc) {
  const std::string& message_type = GetMessageType(channel_name);
  if (message_type.empty()) {
    if (!WriteChannel(channel_name, MessageT::descriptor()->full_name(),
                      proto_desc)) {
      AERROR << "Failed to write meta data to channel [" << channel_name
             << "].";
      return false;
    }
  } else {
    if (MessageT::descriptor()->full_name() != message_type) {
      AERROR << "Message type is invalid, expect: " << message_type
             << ", actual: " << MessageT::descriptor()->full_name();
      return false;
    }
  }
  std::string content("");
  if (!message.SerializeToString(&content)) {
    AERROR << "Failed to serialize message, channel: " << channel_name;
    return false;
  }
  return WriteMessage(channel_name, content, time);
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_RECORD_WRITER_H_
