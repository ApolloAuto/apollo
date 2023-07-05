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

#include "cyber/tools/cyber_recorder/player/play_task.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace record {

std::atomic<uint64_t> PlayTask::played_msg_num_ = {0};

PlayTask::PlayTask(const MessagePtr& msg, const WriterPtr& writer,
                   uint64_t msg_real_time_ns, uint64_t msg_play_time_ns)
    : msg_(msg),
      writer_(writer),
      msg_real_time_ns_(msg_real_time_ns),
      msg_play_time_ns_(msg_play_time_ns) {}

void PlayTask::Play() {
  if (writer_ == nullptr) {
    AERROR << "writer is nullptr, can't write message.";
    return;
  }

  if (!writer_->Write(msg_)) {
    AERROR << "write message failed, played num: " << played_msg_num_.load()
           << ", real time: " << msg_real_time_ns_
           << ", play time: " << msg_play_time_ns_;
    return;
  }

  played_msg_num_.fetch_add(1);

  ADEBUG << "write message succ, played num: " << played_msg_num_.load()
         << ", real time: " << msg_real_time_ns_
         << ", play time: " << msg_play_time_ns_;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
