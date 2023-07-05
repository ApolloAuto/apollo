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

#ifndef CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_BUFFER_H_
#define CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_BUFFER_H_

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>

#include "cyber/tools/cyber_recorder/player/play_task.h"

namespace apollo {
namespace cyber {
namespace record {

class PlayTaskBuffer {
 public:
  using TaskPtr = std::shared_ptr<PlayTask>;
  // if all tasks are in order, we can use other container to replace this
  using TaskMap = std::multimap<uint64_t, TaskPtr>;

  PlayTaskBuffer();
  virtual ~PlayTaskBuffer();

  size_t Size() const;
  bool Empty() const;

  void Push(const TaskPtr& task);
  TaskPtr Front();
  void PopFront();

 private:
  TaskMap tasks_;
  mutable std::mutex mutex_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_BUFFER_H_
