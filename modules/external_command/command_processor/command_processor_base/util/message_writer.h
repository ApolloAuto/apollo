/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

/**
 * @file message_writer.h
 */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <list>

#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/time/clock.h"

namespace apollo {            // namespace apollo
namespace external_command {  // namespace external_command

/**
 * @brief Wrapped Writer with last sent channel.
 */
class WriterHandle {
 public:
  WriterHandle(
      const std::shared_ptr<cyber::WriterBase>& writer,
      const std::shared_ptr<cyber::WriterBase>& history_writer = nullptr)
      : writer_(writer),
        history_writer_(history_writer),
        last_message_(nullptr) {}
  /**
   * @brief Write the message with the channel name.
   * @param message The message to be written.
   * @return true if the message writer is registered;
   */
  template <typename T>
  bool Write(const T& message);
  /**
   * @brief Write the message with the channel name.
   * @param message The message to be written.
   * @return true if the message writer is registered;
   */
  template <typename T>
  bool Write(const std::shared_ptr<T>& message);
  /**
   * @brief Write the last received message as history.
   */
  void WriteLastMessage();

 private:
  template <typename T>
  void WriteHistoryMessage();

  std::shared_ptr<cyber::WriterBase> writer_;
  std::shared_ptr<cyber::WriterBase> history_writer_;
  std::shared_ptr<google::protobuf::Message> last_message_;
  std::function<void()> write_history_function_;
};

template <typename T>
bool WriterHandle::Write(const T& message) {
  auto* message_writer = dynamic_cast<cyber::Writer<T>*>(writer_.get());
  if (nullptr == message_writer) {
    return false;
  }
  message_writer->Write(message);
  if (nullptr == last_message_) {
    last_message_ = std::dynamic_pointer_cast<google::protobuf::Message>(
        std::make_shared<T>());
    write_history_function_ =
        std::bind(&WriterHandle::WriteHistoryMessage<T>, this);
  }
  last_message_->CopyFrom(message);
  return true;
}

/**
 * @brief Write the message with the channel name.
 * @param channel_name Channel name of the message.
 * @param message The message to be written.
 * @return true if the message writer is registered;
 */
template <typename T>
bool WriterHandle::Write(const std::shared_ptr<T>& message) {
  CHECK_NOTNULL(message);
  return Write(*message);
}

void WriterHandle::WriteLastMessage() {
  if (write_history_function_) {
    write_history_function_();
  }
}

template <typename T>
void WriterHandle::WriteHistoryMessage() {
  std::shared_ptr<cyber::Writer<T>> history_writer =
      std::dynamic_pointer_cast<cyber::Writer<T>>(history_writer_);
  if (nullptr == history_writer) {
    ADEBUG << "History Writer type is not correct!";
    return;
  }
  std::shared_ptr<T> message = std::dynamic_pointer_cast<T>(last_message_);
  if (message == nullptr) {
    ADEBUG << "History message type is not correct!";
    return;
  }
  auto timestamp = apollo::cyber::Clock::NowInSeconds();
  message->mutable_header()->set_timestamp_sec(timestamp);
  history_writer->Write(message);
}

class MessageWriter {
 public:
  /**
   * @brief Register the message writer with the given channel name.
   * @param channel_name Channel name of the message.
   * @param history_channel_name if it is set, last received message will be
   * sent periodically.
   */
  template <typename T>
  std::shared_ptr<WriterHandle> RegisterMessage(
      const std::string& channel_name,
      const std::string& history_channel_name = "");
  /**
   * @brief Register the message writer with the given channel atrributes.
   * @param role_attr The channel atrributes.
   * @param history_channel_name if it is set, last received message will be
   * sent periodically.
   */
  template <typename T>
  std::shared_ptr<WriterHandle> RegisterMessage(
      const cyber::proto::RoleAttributes& role_attr,
      const std::string& history_channel_name = "");

 private:
  std::map<std::string, std::shared_ptr<WriterHandle>> writer_map_;
  std::list<std::shared_ptr<WriterHandle>> history_writers_;
  std::shared_ptr<cyber::Node> node_;
  // Timer for sending history message for debug.
  std::unique_ptr<::apollo::cyber::Timer> timer_;

  DECLARE_SINGLETON(MessageWriter)
};

template <typename T>
std::shared_ptr<WriterHandle> MessageWriter::RegisterMessage(
    const std::string& channel_name, const std::string& history_channel_name) {
  auto existing_writer_iter = writer_map_.find(channel_name);
  // Only register writer when it is not registered before.
  if (existing_writer_iter == writer_map_.end()) {
    auto writer = node_->CreateWriter<T>(channel_name);
    if (history_channel_name != "") {
      auto history_writer = node_->CreateWriter<T>(history_channel_name);
      writer_map_[channel_name] =
          std::make_shared<WriterHandle>(writer, history_writer);
      history_writers_.emplace_back(writer_map_[channel_name]);
    } else {
      writer_map_[channel_name] = std::make_shared<WriterHandle>(writer);
    }
    return writer_map_[channel_name];
  }
  return existing_writer_iter->second;
}

template <typename T>
std::shared_ptr<WriterHandle> MessageWriter::RegisterMessage(
    const cyber::proto::RoleAttributes& role_attr,
    const std::string& history_channel_name) {
  const auto& channel_name = role_attr.channel_name();
  auto existing_writer_iter = writer_map_.find(channel_name);
  // Only register writer when it is not registered before.
  if (existing_writer_iter == writer_map_.end()) {
    auto writer = node_->CreateWriter<T>(role_attr);
    if (history_channel_name != "") {
      auto history_writer = node_->CreateWriter<T>(history_channel_name);
      writer_map_[channel_name] =
          std::make_shared<WriterHandle>(writer, history_writer);
      history_writers_.emplace_back(writer_map_[channel_name]);
    } else {
      writer_map_[channel_name] = std::make_shared<WriterHandle>(writer);
    }
    return writer_map_[channel_name];
  }
  return existing_writer_iter->second;
}

}  // namespace external_command
}  // namespace apollo
