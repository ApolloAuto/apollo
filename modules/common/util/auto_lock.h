/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <atomic>
#include <mutex>
#include <stdexcept>

#include "glog/logging.h"

/**
 * @brief MUST implement the following two routine if don't use std::mutex
 */
template<typename T>
void InternalLock(T *t);
template<typename T>
void InternalUnLock(T *t);

/**
 * @brief override these two routines for other synchronization methods.
 */
void InternalLock(std::mutex *t) {
  CHECK(t != nullptr);
  t->lock();
}

void InternalUnLock(std::mutex *t) {
  CHECK(t !=  nullptr);
  t->unlock();
}

template<typename T = std::mutex>
class AutoLock {
 public:
  explicit AutoLock(std::mutex *t) :
    t_(t),
    lock_(false) {
    Lock();
  }

  ~AutoLock() {
    UnLock();
  }

 public:
  void UnLock() {
    CHECK( t != nullptr);
    if (lock_) {
      InternalUnLock(t_);
      lock_ = false;
    }
  }

 private:
  void Lock() {
    CHECK(t != nullptr);
    if (!lock_) {
      InternalLock(t_);
      lock_ = true;
    }
  }

 private:
  T *t_;
  std::atomic_bool lock_;

 private:
  AutoLock &operator= (const AutoLock &rhs);
};
