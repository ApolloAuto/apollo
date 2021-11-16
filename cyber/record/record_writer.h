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

#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

#include "cyber/proto/record.pb.h"

#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/file/record_file_writer.h"
#include "cyber/record/header_builder.h"
#include "cyber/record/record_base.h"

namespace apollo {
namespace cyber {
namespace record {

/**
 * @brief The record writer.
 */
class RecordWriter : public RecordBase {
 public:
  using MessageNumberMap = std::unordered_map<std::string, uint64_t>;
  using MessageTypeMap = std::unordered_map<std::string, std::string>;
  using MessageProtoDescMap = std::unordered_map<std::string, std::string>;
  using FileWriterPtr = std::unique_ptr<RecordFileWriter>;

  /**
   * @brief The default constructor.
   */
  RecordWriter();

  /**
   * @brief The constructor with record header as parameter.
   *
   * @param header
   */
  explicit RecordWriter(const proto::Header& header);

  /**
   * @brief Virtual Destructor.
   */
  virtual ~RecordWriter();

  /**
   * @brief Open a record to write.
   *
   * @param file
   *
   * @return True for success, false for fail.
   */
  bool Open(const std::string& file);

  /**
   * @brief Clean the record.
   */
  void Close();

  /**
   * @brief Write a channel to record.
   *
   * @param channel_name
   * @param message_type
   * @param proto_desc
   *
   * @return True for success, false for fail.
   */
  bool WriteChannel(const std::string& channel_name,
                    const std::string& message_type,
                    const std::string& proto_desc);

  /**
   * @brief Write a message to record.
   *
   * @tparam MessageT
   * @param channel_name
   * @param message
   * @param time_nanosec
   * @param proto_desc
   *
   * @return True for success, false for fail.
   */
  template <typename MessageT>
  bool WriteMessage(const std::string& channel_name, const MessageT& message,
                    const uint64_t time_nanosec,
                    const std::string& proto_desc = "");

  /**
   * @brief Set max size (KB) to segment record file
   *
   * @param size_kilobytes
   *
   * @return True for success, false for fail.
   */
  bool SetSizeOfFileSegmentation(uint64_t size_kilobytes);

  /**
   * @brief Set max interval (Second) to segment record file.
   *
   * @param time_sec
   *
   * @return True for success, false for fail.
   */
  bool SetIntervalOfFileSegmentation(uint64_t time_sec);

  /**
   * @brief Get message number by channel name.
   *
   * @param channel_name
   *
   * @return Message number.
   */
  uint64_t GetMessageNumber(const std::string& channel_name) const override;

  /**
   * @brief Get message type by channel name.
   *
   * @param channel_name
   *
   * @return Message type.
   */
  const std::string& GetMessageType(
      const std::string& channel_name) const override;

  /**
   * @brief Get proto descriptor string by channel name.
   *
   * @param channel_name
   *
   * @return Proto descriptor string by channel name.
   */
  const std::string& GetProtoDesc(
      const std::string& channel_name) const override;

  /**
   * @brief Get channel list.
   *
   * @return List container with all channel name string.
   */
  std::set<std::string> GetChannelList() const override;

  /**
   * @brief Is a new channel recording or not.
   *
   * @return True for yes, false for no.
   */
  bool IsNewChannel(const std::string& channel_name) const;

 private:
  bool WriteMessage(const proto::SingleMessage& single_msg);
  bool SplitOutfile();
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
                                       const std::string& message,
                                       const uint64_t time_nanosec,
                                       const std::string& proto_desc) {
  proto::SingleMessage single_msg;
  single_msg.set_channel_name(channel_name);
  single_msg.set_content(message);
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
    if (!WriteChannel(channel_name, message::GetMessageName<MessageT>(),
                      proto_desc)) {
      AERROR << "Failed to write meta data to channel [" << channel_name
             << "].";
      return false;
    }
  } else {
    if (MessageT::descriptor()->full_name() != message_type) {
      AERROR << "Message type is invalid, expect: " << message_type
             << ", actual: " << message::GetMessageName<MessageT>();
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
