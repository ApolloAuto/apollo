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

const int32_t Block::kRWLockFree = 0;
const int32_t Block::kWriteExclusive = -1;
const int32_t Block::kMaxTryLockTimes = 5;

Block::Block() : msg_size_(0), msg_info_size_(0) {}

Block::~Block() {}

bool Block::TryLockForWrite() {
  int32_t rw_lock_free = kRWLockFree;
  if (!lock_num_.compare_exchange_weak(rw_lock_free, kWriteExclusive,
                                       std::memory_order_acq_rel,
                                       std::memory_order_relaxed)) {
    ADEBUG << "lock num: " << lock_num_.load();
    return false;
  }
  return true;
}

bool Block::TryLockForRead() {
  int32_t lock_num = lock_num_.load();
  if (lock_num < kRWLockFree) {
    AINFO << "block is being written.";
    return false;
  }

  int32_t try_times = 0;
  while (!lock_num_.compare_exchange_weak(lock_num, lock_num + 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
    ++try_times;
    if (try_times == kMaxTryLockTimes) {
      AINFO << "fail to add read lock num, curr num: " << lock_num;
      return false;
    }

    lock_num = lock_num_.load();
    if (lock_num < kRWLockFree) {
      AINFO << "block is being written.";
      return false;
    }
  }

  return true;
}

void Block::ReleaseWriteLock() { lock_num_.fetch_add(1); }

void Block::ReleaseReadLock() { lock_num_.fetch_sub(1); }

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
