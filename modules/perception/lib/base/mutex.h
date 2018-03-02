/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_LIB_BASE_MUTEX_H_
#define MODULES_PERCEPTION_LIB_BASE_MUTEX_H_

#include <pthread.h>

#include "modules/common/macro.h"

namespace apollo {
namespace perception {

class Mutex {
 public:
  Mutex() {
    pthread_mutex_init(&mu_, nullptr);
  }

  ~Mutex() {
    pthread_mutex_destroy(&mu_);
  }

  inline void Lock() {
    pthread_mutex_lock(&mu_);
  }

  inline void Unlock() {
    pthread_mutex_unlock(&mu_);
  }

  inline bool TryLock() {
    return pthread_mutex_trylock(&mu_) == 0;
  }

 private:
  friend class CondVar;

  pthread_mutex_t mu_;

  DISALLOW_COPY_AND_ASSIGN(Mutex);
};

class MutexLock {
 public:
  explicit MutexLock(Mutex *mu) : mu_(mu) {
    mu_->Lock();
  }
  ~MutexLock() {
    mu_->Unlock();
  }

 private:
  Mutex *const mu_;
  DISALLOW_COPY_AND_ASSIGN(MutexLock);
};

// Wrapper for pthread_cond_t
class CondVar {
 public:
  CondVar() {
    pthread_cond_init(&cv_, nullptr);
  }

  ~CondVar() {
    pthread_cond_destroy(&cv_);
  }

  void Wait(Mutex *mu) {
    pthread_cond_wait(&cv_, &mu->mu_);
  }

  void Signal() {
    pthread_cond_signal(&cv_);
  }

  void Signalall() {
    pthread_cond_broadcast(&cv_);
  }

 private:
  pthread_cond_t cv_;
  DISALLOW_COPY_AND_ASSIGN(CondVar);
};

class BlockingCounter {
 public:
  explicit BlockingCounter(size_t cnt) : counter_(cnt) {}

  bool Decrement() {
    MutexLock lock(&mutex_);
    --counter_;

    if (counter_ == 0u) {
      cond_.Signalall();
    }

    return counter_ == 0u;
  }

  void Reset(size_t cnt) {
    MutexLock lock(&mutex_);
    counter_ = cnt;
  }

  void Wait() {
    MutexLock lock(&mutex_);

    while (counter_ != 0u) {
      cond_.Wait(&mutex_);
    }
  }

 private:
  Mutex mutex_;
  CondVar cond_;
  size_t counter_;
  DISALLOW_COPY_AND_ASSIGN(BlockingCounter);
};

class RwMutex {
 public:
  RwMutex() {
    pthread_rwlock_init(&mu_, nullptr);
  }
  ~RwMutex() {
    pthread_rwlock_destroy(&mu_);
  }

  inline void ReaderLock() {
    pthread_rwlock_rdlock(&mu_);
  }
  inline void WriterLock() {
    pthread_rwlock_wrlock(&mu_);
  }

  inline void Unlock() {
    pthread_rwlock_unlock(&mu_);
  }

 private:
  pthread_rwlock_t mu_;
  DISALLOW_COPY_AND_ASSIGN(RwMutex);
};

class ReaderMutexLock {
 public:
  explicit ReaderMutexLock(RwMutex *mu) : mu_(mu) {
    mu_->ReaderLock();
  }
  ~ReaderMutexLock() {
    mu_->Unlock();
  }

 private:
  RwMutex *mu_ = nullptr;
  DISALLOW_COPY_AND_ASSIGN(ReaderMutexLock);
};

class WriterMutexLock {
 public:
  explicit WriterMutexLock(RwMutex *mu) : mu_(mu) {
    mu_->WriterLock();
  }
  ~WriterMutexLock() {
    mu_->Unlock();
  }

 private:
  RwMutex *mu_ = nullptr;
  DISALLOW_COPY_AND_ASSIGN(WriterMutexLock);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_LIB_BASE_MUTEX_H_
