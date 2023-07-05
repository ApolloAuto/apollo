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

#ifndef CYBER_RECORD_RECORD_BASE_H_
#define CYBER_RECORD_RECORD_BASE_H_

#include <cstdint>
#include <set>
#include <string>

#include "cyber/proto/record.pb.h"

namespace apollo {
namespace cyber {
namespace record {

static const std::string& kEmptyString = "";

/**
 * @brief Base class for record reader and writer.
 */
class RecordBase {
 public:
  /**
   * @brief Destructor.
   */
  virtual ~RecordBase() = default;

  /**
   * @brief Get message number by channel name.
   *
   * @param channel_name
   *
   * @return Message number.
   */
  virtual uint64_t GetMessageNumber(const std::string& channel_name) const = 0;

  /**
   * @brief Get message type by channel name.
   *
   * @param channel_name
   *
   * @return Message type.
   */
  virtual const std::string& GetMessageType(
      const std::string& channel_name) const = 0;

  /**
   * @brief Get proto descriptor string by channel name.
   *
   * @param channel_name
   *
   * @return Proto descriptor string by channel name.
   */
  virtual const std::string& GetProtoDesc(
      const std::string& channel_name) const = 0;

  /**
   * @brief Get channel list.
   *
   * @return List container with all channel name string.
   */
  virtual std::set<std::string> GetChannelList() const = 0;

  /**
   * @brief Get record header.
   *
   * @return Record header.
   */
  const proto::Header& GetHeader() const { return header_; }

  /**
   * @brief Get record file path.
   *
   * @return Record file path.
   */
  const std::string GetFile() const { return file_; }

 protected:
  std::string file_;
  proto::Header header_;
  bool is_opened_ = false;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_RECORD_BASE_H_
