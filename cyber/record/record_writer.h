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

#ifndef CYBER_RECORD_RECORD_WRITER_H_
#define CYBER_RECORD_RECORD_WRITER_H_

#include <stdint.h>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include "cyber/common/log.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_writer.h"
#include "cyber/record/header_builder.h"
#include "cyber/record/record_base.h"

namespace apollo {
namespace cyber {
namespace record {

class RecordWriter : public RecordBase {
 public:
  using MessageNumberMap = std::unordered_map<std::string, uint64_t>;
  using MessageTypeMap = std::unordered_map<std::string, std::string>;
  using MessageProtoDescMap = std::unordered_map<std::string, std::string>;
  using FileWriterPtr = std::unique_ptr<RecordFileWriter>;

  RecordWriter();

  virtual ~RecordWriter();

  bool Open(const std::string& file);

  void Close();

  bool WriteChannel(const std::string& channel_name,
                    const std::string& message_type,
                    const std::string& proto_desc);

  template <typename MessageT>
  bool WriteMessage(const std::string& channel_name, const MessageT& message,
                    const uint64_t time_nanosec,
                    const std::string& proto_desc = "");

  bool SetSizeOfFileSegmentation(uint64_t size_kilobytes);

  bool SetIntervalOfFileSegmentation(uint64_t time_sec);

  uint64_t GetMessageNumber(const std::string& channel_name) const override;

  const std::string& GetMessageType(
      const std::string& channel_name) const override;

  const std::string& GetProtoDesc(
      const std::string& channel_name) const override;

 private:
  bool WriteMessage(const proto::SingleMessage& single_msg);
  bool SplitOutfile();
  bool IsNewChannel(const std::string& channel_name);
  void OnNewChannel(const std::string& channel_name,
                    const std::string& message_type,
                    const std::string& proto_desc);
  void OnNewMessage(const std::string& channel_name);

  std::string path_;
  uint64_t segment_raw_size_ = 0;
  uint64_t segment_begin_time_ = 0;
  uint32_t file_index_ = 0;
  MessageNumberMap channel_message_number_map_;
  MessageTypeMap channel_message_type_map_;
  MessageProtoDescMap channel_proto_desc_map_;
  FileWriterPtr file_writer_ = nullptr;
  FileWriterPtr file_writer_backup_ = nullptr;
  std::mutex mutex_;
  std::stringstream sstream_;
};

template <>
inline bool RecordWriter::WriteMessage(const std::string& channel_name,
                                       const std::string& content,
                                       const uint64_t time_nanosec,
                                       const std::string& proto_desc) {
  proto::SingleMessage single_msg;
  single_msg.set_channel_name(channel_name);
  single_msg.set_content(content);
  single_msg.set_time(time_nanosec);
  return WriteMessage(single_msg);
}

template <>
inline bool RecordWriter::WriteMessage(
    const std::string& channel_name,
    const std::shared_ptr<message::RawMessage>& message,
    const uint64_t time_nanosec, const std::string& proto_desc) {
  if (message == nullptr) {
    AERROR << "nullptr error, channel: " << channel_name;
    return false;
  }
  return WriteMessage(channel_name, message->message, time_nanosec);
}

template <typename MessageT>
bool RecordWriter::WriteMessage(const std::string& channel_name,
                                const MessageT& message,
                                const uint64_t time_nanosec,
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
  return WriteMessage(channel_name, content, time_nanosec);
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_RECORD_WRITER_H_
