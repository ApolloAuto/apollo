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

#ifndef CYBER_BASE_REENTRANT_RW_LOCK_H_
#define CYBER_BASE_REENTRANT_RW_LOCK_H_

#include <unistd.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

#include "cyber/base/rw_lock_guard.h"

namespace apollo {
namespace cyber {
namespace base {

static const std::thread::id NULL_THREAD_ID = std::thread::id();
class ReentrantRWLock {
  friend class ReadLockGuard<ReentrantRWLock>;
  friend class WriteLockGuard<ReentrantRWLock>;

 public:
  static const int32_t RW_LOCK_FREE = 0;
  static const int32_t WRITE_EXCLUSIVE = -1;
  static const uint32_t MAX_RETRY_TIMES = 5;
  static const std::thread::id null_thread;
  ReentrantRWLock() {}
  explicit ReentrantRWLock(bool write_first) : write_first_(write_first) {}

 private:
  // all these function only can used by ReadLockGuard/WriteLockGuard;
  void ReadLock();
  void WriteLock();

  void ReadUnlock();
  void WriteUnlock();

  ReentrantRWLock(const ReentrantRWLock&) = delete;
  ReentrantRWLock& operator=(const ReentrantRWLock&) = delete;
  std::thread::id write_thread_id_ = {NULL_THREAD_ID};
  std::atomic<uint32_t> write_lock_wait_num_ = {0};
  std::atomic<int32_t> lock_num_ = {0};
  bool write_first_ = true;
};

inline void ReentrantRWLock::ReadLock() {
  if (write_thread_id_ == std::this_thread::get_id()) {
    return;
  }

  uint32_t retry_times = 0;
  int32_t lock_num = lock_num_.load(std::memory_order_acquire);
  if (write_first_) {
    do {
      while (lock_num < RW_LOCK_FREE ||
             write_lock_wait_num_.load(std::memory_order_acquire) > 0) {
        if (++retry_times == MAX_RETRY_TIMES) {
          // saving cpu
          std::this_thread::yield();
          retry_times = 0;
        }
        lock_num = lock_num_.load(std::memory_order_acquire);
      }
    } while (!lock_num_.compare_exchange_weak(lock_num, lock_num + 1,
                                              std::memory_order_acq_rel,
                                              std::memory_order_relaxed));
  } else {
    do {
      while (lock_num < RW_LOCK_FREE) {
        if (++retry_times == MAX_RETRY_TIMES) {
          // saving cpu
          std::this_thread::yield();
          retry_times = 0;
        }
        lock_num = lock_num_.load(std::memory_order_acquire);
      }
    } while (!lock_num_.compare_exchange_weak(lock_num, lock_num + 1,
                                              std::memory_order_acq_rel,
                                              std::memory_order_relaxed));
  }
}

inline void ReentrantRWLock::WriteLock() {
  auto this_thread_id = std::this_thread::get_id();
  if (write_thread_id_ == this_thread_id) {
    lock_num_.fetch_sub(1);
    return;
  }
  int32_t rw_lock_free = RW_LOCK_FREE;
  uint32_t retry_times = 0;
  write_lock_wait_num_.fetch_add(1);
  while (!lock_num_.compare_exchange_weak(rw_lock_free, WRITE_EXCLUSIVE,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed)) {
    // rw_lock_free will change after CAS fail, so init agin
    rw_lock_free = RW_LOCK_FREE;
    if (++retry_times == MAX_RETRY_TIMES) {
      // saving cpu
      std::this_thread::yield();
      retry_times = 0;
    }
  }
  write_thread_id_ = this_thread_id;
  write_lock_wait_num_.fetch_sub(1);
}

inline void ReentrantRWLock::ReadUnlock() {
  if (write_thread_id_ == std::this_thread::get_id()) {
    return;
  }
  lock_num_.fetch_sub(1);
}

inline void ReentrantRWLock::WriteUnlock() {
  if (lock_num_.fetch_add(1) == WRITE_EXCLUSIVE) {
    write_thread_id_ = NULL_THREAD_ID;
  }
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_REENTRANT_RW_LOCK_H_
