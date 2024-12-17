/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#ifndef CYBER_BASE_PTHREAD_RW_LOCK_H_
#define CYBER_BASE_PTHREAD_RW_LOCK_H_

#include <thread>

#include "cyber/base/rw_lock_guard.h"

namespace apollo {
namespace cyber {
namespace base {

class PthreadRWLock {
  friend class ReadLockGuard<PthreadRWLock>;
  friend class WriteLockGuard<PthreadRWLock>;

 public:
  explicit PthreadRWLock(bool writer) {
    pthread_rwlockattr_init(&rwlock_attr_);
    if (writer) {
      pthread_rwlockattr_setkind_np(
          &rwlock_attr_, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP);
    }
    pthread_rwlockattr_setpshared(&rwlock_attr_, PTHREAD_PROCESS_SHARED);
    pthread_rwlock_init(&rwlock_, &rwlock_attr_);
  }
  PthreadRWLock() : PthreadRWLock(true) {}

  ~PthreadRWLock() {
    pthread_rwlock_destroy(&rwlock_);
    pthread_rwlockattr_destroy(&rwlock_attr_);
  }

  void ReadLock();
  void ReadUnlock();

  void WriteLock();
  void WriteUnlock();

 private:
  PthreadRWLock(const PthreadRWLock& other) = delete;
  PthreadRWLock& operator=(const PthreadRWLock& other) = delete;
  pthread_rwlock_t rwlock_;
  pthread_rwlockattr_t rwlock_attr_;
};

inline void PthreadRWLock::ReadLock() { pthread_rwlock_rdlock(&rwlock_); }

inline void PthreadRWLock::ReadUnlock() { pthread_rwlock_unlock(&rwlock_); }

inline void PthreadRWLock::WriteLock() { pthread_rwlock_wrlock(&rwlock_); }

inline void PthreadRWLock::WriteUnlock() { pthread_rwlock_unlock(&rwlock_); }

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif
