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

#include <stdint.h>
#include <string>

#include "cyber/proto/record.pb.h"

namespace apollo {
namespace cyber {
namespace record {

class RecordBase {
 public:
  virtual ~RecordBase() = default;

  virtual uint64_t GetMessageNumber(const std::string& channel_name) const = 0;

  virtual const std::string& GetMessageType(
      const std::string& channel_name) const = 0;

  virtual const std::string& GetProtoDesc(
      const std::string& channel_name) const = 0;

  const proto::Header& GetHeader() const { return header_; }

 protected:
  std::string file_;
  std::string path_;
  std::string null_type_;
  proto::Header header_;
  bool is_opened_ = false;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_RECORD_RECORD_BASE_H_
