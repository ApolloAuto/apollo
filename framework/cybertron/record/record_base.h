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

#ifndef CYBERTRON_RECORD_RECORD_BASE_H_
#define CYBERTRON_RECORD_RECORD_BASE_H_

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "cybertron/common/file.h"
#include "cybertron/common/log.h"
#include "cybertron/record/record_file.h"

namespace apollo {
namespace cybertron {
namespace record {

static std::string g_empty_string = "";

class RecordBase {
 public:
  RecordBase();
  virtual ~RecordBase();
  uint64_t GetMessageNumber(const std::string& channel_name) const;
  const std::string& GetMessageType(const std::string& channel_name) const;
  const std::string& GetProtoDesc(const std::string& channel_name) const;
  const Header GetHeader() const;

 protected:
  void OnNewChannel(const std::string& channel_name,
                    const std::string& message_type,
                    const std::string& proto_desc);

  void OnNewMessage(const std::string& channel_name);

  Header header_;
  std::mutex mutex_;
  std::string file_;
  std::string path_;
  std::unordered_map<std::string, uint64_t> channel_message_number_map_;
  std::unordered_map<std::string, std::string> channel_message_type_map_;
  std::unordered_map<std::string, std::string> channel_proto_desc_map_;
  bool is_opened_ = false;
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_RECORD_BASE_H_
