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

#include "cyber/profiler/block.h"

#include "cyber/profiler/block_manager.h"

namespace apollo {
namespace cyber {
namespace profiler {

Block::Block() : depth_(0), begin_time_(), end_time_() {}

Block::Block(const std::string& name)
    : name_(name), depth_(0), begin_time_(), end_time_() {}

Block::~Block() {
  if (!finished())
    BlockManager::Instance()->EndBlock();
}

void Block::Start() {
  begin_time_ = std::chrono::steady_clock::now();
}

void Block::End() {
  end_time_ = std::chrono::steady_clock::now();
}

std::uint64_t Block::begin_time_since_epoch() const {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             begin_time_.time_since_epoch()).count();
}

std::uint64_t Block::end_time_since_epoch() const {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             end_time_.time_since_epoch()).count();
}

std::uint64_t Block::duration() const {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
            end_time_ - begin_time_).count();
}

}  // namespace profiler
}  // namespace cyber
}  // namespace apollo
