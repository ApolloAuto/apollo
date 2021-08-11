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

#ifndef CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_CONSUMER_H_
#define CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_CONSUMER_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "cyber/tools/cyber_recorder/player/play_task_buffer.h"

namespace apollo {
namespace cyber {
namespace record {

class PlayTaskConsumer {
 public:
  using ThreadPtr = std::unique_ptr<std::thread>;
  using TaskBufferPtr = std::shared_ptr<PlayTaskBuffer>;

  explicit PlayTaskConsumer(const TaskBufferPtr& task_buffer,
                            double play_rate = 1.0);
  virtual ~PlayTaskConsumer();

  void Start(uint64_t begin_time_ns);
  void Stop();
  void Pause() { is_paused_.exchange(true); }
  void PlayOnce() { is_playonce_.exchange(true); }
  void Continue() { is_paused_.exchange(false); }

  uint64_t base_msg_play_time_ns() const { return base_msg_play_time_ns_; }
  uint64_t base_msg_real_time_ns() const { return base_msg_real_time_ns_; }
  uint64_t last_played_msg_real_time_ns() const {
    return last_played_msg_real_time_ns_;
  }

 private:
  void ThreadFunc();

  double play_rate_;
  ThreadPtr consume_th_;
  TaskBufferPtr task_buffer_;
  std::atomic<bool> is_stopped_;
  std::atomic<bool> is_paused_;
  std::atomic<bool> is_playonce_;
  uint64_t begin_time_ns_;
  uint64_t base_msg_play_time_ns_;
  uint64_t base_msg_real_time_ns_;
  uint64_t last_played_msg_real_time_ns_;
  static const uint64_t kPauseSleepNanoSec;
  static const uint64_t kWaitProduceSleepNanoSec;
  static const uint64_t MIN_SLEEP_DURATION_NS;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_CONSUMER_H_
