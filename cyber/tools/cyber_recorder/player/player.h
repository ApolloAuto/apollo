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

#ifndef CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAYER_H_
#define CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAYER_H_

#include <atomic>
#include <memory>
#include <thread>

#include "cyber/tools/cyber_recorder/player/play_param.h"
#include "cyber/tools/cyber_recorder/player/play_task_buffer.h"
#include "cyber/tools/cyber_recorder/player/play_task_consumer.h"
#include "cyber/tools/cyber_recorder/player/play_task_producer.h"

namespace apollo {
namespace cyber {
namespace record {

class Player {
 public:
  using ConsumerPtr = std::unique_ptr<PlayTaskConsumer>;
  using ProducerPtr = std::unique_ptr<PlayTaskProducer>;
  using TaskBufferPtr = std::shared_ptr<PlayTaskBuffer>;

  explicit Player(const PlayParam& play_param);
  virtual ~Player();

  bool Init();
  bool Start();
  bool Stop();

 private:
  void ThreadFunc_Term();

 private:
  std::atomic<bool> is_initialized_ = {false};
  std::atomic<bool> is_stopped_ = {false};
  std::atomic<bool> is_paused_ = {false};
  std::atomic<bool> is_playonce_ = {false};
  ConsumerPtr consumer_;
  ProducerPtr producer_;
  TaskBufferPtr task_buffer_;
  std::shared_ptr<std::thread> term_thread_ = nullptr;
  static const uint64_t kSleepIntervalMiliSec;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAYER_H_
