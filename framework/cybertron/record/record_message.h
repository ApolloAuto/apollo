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

#ifndef CYBERTRON_RECORD_RECORD_MESSAGE_H_
#define CYBERTRON_RECORD_RECORD_MESSAGE_H_

#include <condition_variable>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "cybertron/message/raw_message.h"
#include "cybertron/record/record_base.h"

namespace apollo {
namespace cybertron {
namespace record {

using ::apollo::cybertron::message::RawMessage;
using ::apollo::cybertron::record::RecordFileReader;

struct RecordMessage {
  RecordMessage() {}
  RecordMessage(const std::string& name, 
                const std::string& message,
                uint64_t msg_time)
      : channel_name(name),
        content(message), 
        time(msg_time) {}
  std::string channel_name;
  std::string content;
  uint64_t time;
};

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_RECORD_RECORD_READER_H_
