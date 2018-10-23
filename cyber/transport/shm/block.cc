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

#include "cyber/transport/shm/block.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

Block::Block()
    : is_writing_(false),
      reading_reference_counts_(0),
      msg_size_(0),
      msg_info_size_(0) {}

Block::~Block() {}

bool Block::TryLockForWrite() {
  WriteLockGuard<AtomicRWLock> lock(read_write_mutex_);
  if (is_writing_.load()) {
    ADEBUG << "block is writing.";
    return false;
  } else if (reading_reference_counts_.load() != 0) {
    ADEBUG << "block is reading.";
    return false;
  }
  is_writing_.store(true);
  return true;
}

bool Block::TryLockForRead() {
  ReadLockGuard<AtomicRWLock> lock(read_write_mutex_);
  if (is_writing_.load()) {
    ADEBUG << "block is writing.";
    return false;
  }
  reading_reference_counts_.fetch_add(1);
  return true;
}

void Block::ReleaseWriteLock() { is_writing_.store(false); }

void Block::ReleaseReadLock() { reading_reference_counts_.fetch_sub(1); }

// for radical write, we should call TryLockForWrite() firstly
bool Block::Write(uint8_t* dst, const std::string& msg,
                  const std::string& msg_info) {
  msg_size_ = msg.length();
  msg_info_size_ = msg_info.length();
  std::memcpy(dst, const_cast<char*>(msg.data()), msg_size_);
  std::memcpy(dst + msg_size_, const_cast<char*>(msg_info.data()),
              msg_info_size_);
  return true;
}

// for radical read, we should call TryLockForRead() firstly
bool Block::Read(const uint8_t* src, std::string* msg, std::string* msg_info) {
  if (msg == nullptr || msg_info == nullptr) {
    return false;
  }
  const char* ptr = (const char*)(src);
  msg->assign(ptr, msg_size_);
  ptr += msg_size_;
  msg_info->assign(ptr, msg_info_size_);
  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
