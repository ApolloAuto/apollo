/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "cyber/profiler/frame.h"

#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace profiler {

constexpr char kModuleName[] = "perf";

void Frame::Push(Block* block) {
  if (block != nullptr)
    stack_.push(block);
}

Block* Frame::Top() {
  if (stack_.empty())
    return nullptr;
  return stack_.top();
}

void Frame::Pop() {
  if (stack_.empty())
    return;

  Block* block_ptr = stack_.top();
  stack_.pop();
  storage_.push_back(std::move(*block_ptr));
}

bool Frame::DumpToFile(const std::string& routine_name) {
  // Use 'ALOG_MODULE' instead of 'AINFO' to specify the log file
  ALOG_MODULE(kModuleName, INFO) << "Frame : " << routine_name;
  for (const Block& block : storage_) {
    ALOG_MODULE(kModuleName, INFO)
        << block.depth() << "," << block.name() << "," << block.duration()
        << "," << block.begin_time_since_epoch()
        << "," << block.end_time_since_epoch();
  }
  return true;
}

void Frame::Clear() {
  storage_.clear();
}

}  // namespace profiler
}  // namespace cyber
}  // namespace apollo
