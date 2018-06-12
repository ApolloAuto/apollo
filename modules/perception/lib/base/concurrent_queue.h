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

#ifndef MODULES_PERCEPTION_LIB_BASE_CONCURRENT_QUEUE_H_
#define MODULES_PERCEPTION_LIB_BASE_CONCURRENT_QUEUE_H_

#include <queue>

#include "modules/perception/lib/base/mutex.h"

namespace apollo {
namespace perception {

template <class Data>
class ConcurrentQueue {
 public:
  ConcurrentQueue() {}
  virtual ~ConcurrentQueue() {}

  virtual void push(const Data& data) {
    MutexLock lock(&mutex_);
    queue_.push(data);
    condition_variable_.Signal();
  }

  virtual void pop(Data* data) {
    MutexLock lock(&mutex_);

    while (queue_.empty()) {
      condition_variable_.Wait(&mutex_);
    }
    *data = queue_.front();
    queue_.pop();
  }

  bool try_pop(Data* data) {
    MutexLock lock(&mutex_);

    if (queue_.empty()) {
      return false;
    }

    *data = queue_.front();
    queue_.pop();
    return true;
  }

  bool empty() {
    MutexLock lock(&mutex_);
    return queue_.empty();
  }

  int size() {
    MutexLock lock(&mutex_);
    return queue_.size();
  }

  void clear() {
    MutexLock lock(&mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

 protected:
  std::queue<Data> queue_;
  Mutex mutex_;
  CondVar condition_variable_;

 private:
  DISALLOW_COPY_AND_ASSIGN(ConcurrentQueue);
};

template <typename Data>
class FixedSizeConQueue : public ConcurrentQueue<Data> {
 public:
  explicit FixedSizeConQueue(size_t max_count)
      : ConcurrentQueue<Data>(), max_count_(max_count) {}

  virtual ~FixedSizeConQueue() {}

  virtual void push(const Data& data) {
    MutexLock lock(&this->mutex_);
    while (this->queue_.size() >= max_count_) {
      condition_full_.Wait(&this->mutex_);
    }
    this->queue_.push(data);
    this->condition_variable_.Signal();
  }

  virtual bool try_push(const Data& data) {
    MutexLock lock(&this->mutex_);
    if (this->queue_.size() >= max_count_) {
      return false;
    }
    this->queue_.push(data);
    this->condition_variable_.Signal();
    return true;
  }

  virtual void pop(Data* data) {
    MutexLock lock(&this->mutex_);

    while (this->queue_.empty()) {
      this->condition_variable_.Wait(&this->mutex_);
    }
    *data = this->queue_.front();
    this->queue_.pop();
    condition_full_.Signal();
  }

  virtual bool try_pop(Data* data) {
    MutexLock lock(&this->mutex_);

    if (this->queue_.empty()) {
      return false;
    }

    *data = this->queue_.front();
    this->queue_.pop();
    condition_full_.Signal();
    return true;
  }

  bool full() const {
    return this->queue_.size() >= max_count_;
  }

 private:
  CondVar condition_full_;
  const size_t max_count_;
  DISALLOW_COPY_AND_ASSIGN(FixedSizeConQueue);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_LIB_BASE_CONCURRENT_QUEUE_H_
