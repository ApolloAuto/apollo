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

#include "cyber/common/macros.h"
#include "cyber/cyber.h"

namespace apollo {            // namespace apollo
namespace external_command {  // namespace external_command

class MessageWriter {
 public:
  /**
   * @brief Register the message writer with the given channel name.
   * @param channel_name Channel name of the message.
   */
  template <typename T>
  std::shared_ptr<apollo::cyber::Writer<T>> RegisterMessage(
      const std::string& channel_name);
  /**
   * @brief Register the message writer with the given channel atrributes.
   * @param role_attr The channel atrributes.
   */
  template <typename T>
  std::shared_ptr<apollo::cyber::Writer<T>> RegisterMessage(
      const cyber::proto::RoleAttributes& role_attr);
  /**
   * @brief Write the message with the channel name.
   * @param channel_name Channel name of the message.
   * @param message The message to be written.
   * @return true if the message writer is registered;
   */
  template <typename T>
  bool WriteMessage(const std::string& channel_name, const T& message) const;
  /**
   * @brief Write the message with the channel name.
   * @param channel_name Channel name of the message.
   * @param message The message to be written.
   * @return true if the message writer is registered;
   */
  template <typename T>
  bool WriteMessage(const std::string& channel_name,
                    const std::shared_ptr<T>& message) const;

 private:
  std::map<std::string, std::shared_ptr<cyber::WriterBase>> writer_map_;
  std::shared_ptr<cyber::Node> node_;

  DECLARE_SINGLETON(MessageWriter)
};

template <typename T>
std::shared_ptr<apollo::cyber::Writer<T>> MessageWriter::RegisterMessage(
    const std::string& channel_name) {
  auto existing_writer_iter = writer_map_.find(channel_name);
  // Only register writer when it is not registered before.
  if (existing_writer_iter == writer_map_.end()) {
    auto writer = node_->CreateWriter<T>(channel_name);
    writer_map_[channel_name] = writer;
    return writer;
  }
  auto existing_writer_ptr = dynamic_cast<apollo::cyber::Writer<T>*>(
      existing_writer_iter->second.get());
  if (nullptr == existing_writer_ptr) {
    return nullptr;
  }
  auto existing_writer =
      std::shared_ptr<apollo::cyber::Writer<T>>(existing_writer_ptr);
  return existing_writer;
}

template <typename T>
std::shared_ptr<apollo::cyber::Writer<T>> MessageWriter::RegisterMessage(
    const cyber::proto::RoleAttributes& role_attr) {
  const auto& channel_name = role_attr.channel_name();
  auto existing_writer_iter = writer_map_.find(channel_name);
  // Only register writer when it is not registered before.
  if (existing_writer_iter == writer_map_.end()) {
    auto writer = node_->CreateWriter<T>(role_attr);
    writer_map_[channel_name] = writer;
    return writer;
  }
  auto existing_writer_ptr = dynamic_cast<apollo::cyber::Writer<T>*>(
      existing_writer_iter->second.get());
  if (nullptr == existing_writer_ptr) {
    return nullptr;
  }
  auto existing_writer =
      std::shared_ptr<apollo::cyber::Writer<T>>(existing_writer_ptr);
  return existing_writer;
}

template <typename T>
bool MessageWriter::WriteMessage(const std::string& channel_name,
                                 const T& message) const {
  if (writer_map_.find(channel_name) == writer_map_.end()) {
    return false;
  }
  auto* base_writer = writer_map_.at(channel_name).get();
  auto* message_writer = dynamic_cast<cyber::Writer<T>*>(base_writer);
  if (nullptr == message_writer) {
    return false;
  }
  message_writer->Write(message);
  return true;
}

/**
 * @brief Write the message with the channel name.
 * @param channel_name Channel name of the message.
 * @param message The message to be written.
 * @return true if the message writer is registered;
 */
template <typename T>
bool MessageWriter::WriteMessage(const std::string& channel_name,
                                 const std::shared_ptr<T>& message) const {
  CHECK_NOT_NULL(message);
  return WriteMessage(*message);
}

}  // namespace external_command
}  // namespace apollo
