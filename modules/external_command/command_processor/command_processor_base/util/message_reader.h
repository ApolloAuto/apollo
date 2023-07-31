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
 * @file message_reader.h
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

class MessageReader {
 public:
  /**
   * @brief Register the message reader with the given channel name.
   * @param channel_name Channel name of the message.
   */
  template <typename T>
  void RegisterMessage(const std::string& channel_name);
  /**
   * @brief Get the message with the channel name.
   * @param channel_name Channel name of the message.
   * @return Message with the channel name; return null if no message with the
   * given type and channel name is registered.
   */
  template <typename T>
  const T* GetMessage(const std::string& channel_name) const;

 private:
  std::map<std::string, std::shared_ptr<cyber::ReaderBase>> reader_map_;
  std::shared_ptr<cyber::Node> node_;

  DECLARE_SINGLETON(MessageReader)
};

template <typename T>
void MessageReader::RegisterMessage(const std::string& channel_name) {
  // Only register reader when it is not registered before.
  if (reader_map_.find(channel_name) == reader_map_.end()) {
    auto reader = node_->CreateReader<T>(channel_name);
    reader_map_[channel_name] = reader;
  }
}

template <typename T>
const T* MessageReader::GetMessage(const std::string& channel_name) const {
  if (reader_map_.find(channel_name) == reader_map_.end()) {
    return nullptr;
  }
  auto* base_reader = reader_map_.at(channel_name).get();
  auto* message_reader = dynamic_cast<cyber::Reader<T>*>(base_reader);
  if (nullptr == message_reader) {
    return nullptr;
  }
  message_reader->Observe();
  if (message_reader->Empty()) {
    AERROR << "Failed to get message of " << channel_name;
    return nullptr;
  }
  const auto* message = message_reader->GetLatestObserved().get();
  return message;
}

}  // namespace external_command
}  // namespace apollo
