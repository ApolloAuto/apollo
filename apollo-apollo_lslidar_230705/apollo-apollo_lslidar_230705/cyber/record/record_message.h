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

#ifndef CYBER_RECORD_RECORD_MESSAGE_H_
#define CYBER_RECORD_RECORD_MESSAGE_H_

#include <cstdint>
#include <string>

namespace apollo {
namespace cyber {
namespace record {

static constexpr size_t kGB = 1 << 30;
static constexpr size_t kMB = 1 << 20;
static constexpr size_t kKB = 1 << 10;

/**
 * @brief Basic data struct of record message.
 */
struct RecordMessage {
  /**
   * @brief The constructor.
   */
  RecordMessage() {}

  /**
   * @brief The constructor.
   *
   * @param name
   * @param message
   * @param msg_time
   */
  RecordMessage(const std::string& name, const std::string& message,
                uint64_t msg_time)
      : channel_name(name), content(message), time(msg_time) {}

  /**
   * @brief The channel name of the message.
   */
  std::string channel_name;

  /**
   * @brief The content of the message.
   */
  std::string content;

  /**
   * @brief The time (nanosecond) of the message.
   */
  uint64_t time;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_RECORD_READER_H_
