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

#ifndef CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_H_
#define CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "cyber/message/raw_message.h"
#include "cyber/node/writer.h"

namespace apollo {
namespace cyber {
namespace record {

class PlayTask {
 public:
  using MessagePtr = std::shared_ptr<message::RawMessage>;
  using WriterPtr = std::shared_ptr<Writer<message::RawMessage>>;

  PlayTask(const MessagePtr& msg, const WriterPtr& writer,
           uint64_t msg_real_time_ns, uint64_t msg_play_time_ns);
  virtual ~PlayTask() {}

  void Play();

  uint64_t msg_real_time_ns() const { return msg_real_time_ns_; }
  uint64_t msg_play_time_ns() const { return msg_play_time_ns_; }
  static uint64_t played_msg_num() { return played_msg_num_.load(); }

 private:
  MessagePtr msg_;
  WriterPtr writer_;
  uint64_t msg_real_time_ns_;
  uint64_t msg_play_time_ns_;

  static std::atomic<uint64_t> played_msg_num_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_H_
