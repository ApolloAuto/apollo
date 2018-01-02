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

#ifndef apollo_PERCEPTION_LIB_BASE_CONCURRENT_QUEUE_H
#define apollo_PERCEPTION_LIB_BASE_CONCURRENT_QUEUE_H

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
    MutexLock lock(&_mutex);
    _queue.push(data);
    _condition_variable.Signal();
  }

  virtual void pop(Data* data) {
    MutexLock lock(&_mutex);

    while (_queue.empty()) {
      _condition_variable.Wait(&_mutex);
    }
    *data = _queue.front();
    _queue.pop();
  }

  bool try_pop(Data* data) {
    MutexLock lock(&_mutex);

    if (_queue.empty()) {
      return false;
    }

    *data = _queue.front();
    _queue.pop();
    return true;
  }

  bool empty() {
    MutexLock lock(&_mutex);
    return _queue.empty();
  }

  int size() {
    MutexLock lock(&_mutex);
    return _queue.size();
  }

  void clear() {
    MutexLock lock(&_mutex);
    while (!_queue.empty()) {
      _queue.pop();
    }
  }

 protected:
  std::queue<Data> _queue;
  Mutex _mutex;
  CondVar _condition_variable;

 private:
  DISALLOW_COPY_AND_ASSIGN(ConcurrentQueue);
};

template <typename Data>
class FixedSizeConQueue : public ConcurrentQueue<Data> {
 public:
  explicit FixedSizeConQueue(size_t max_count)
      : ConcurrentQueue<Data>(), _max_count(max_count) {}

  virtual ~FixedSizeConQueue() {}

  virtual void push(const Data& data) {
    MutexLock lock(&this->_mutex);
    while (this->_queue.size() >= _max_count) {
      _condition_full.Wait(&this->_mutex);
    }
    this->_queue.push(data);
    this->_condition_variable.Signal();
  }

  virtual bool try_push(const Data& data) {
    MutexLock lock(&this->_mutex);
    if (this->_queue.size() >= _max_count) {
      return false;
    }
    this->_queue.push(data);
    this->_condition_variable.Signal();
    return true;
  }

  virtual void pop(Data* data) {
    MutexLock lock(&this->_mutex);

    while (this->_queue.empty()) {
      this->_condition_variable.Wait(&this->_mutex);
    }
    *data = this->_queue.front();
    this->_queue.pop();
    _condition_full.Signal();
  }

  virtual bool try_pop(Data* data) {
    MutexLock lock(&this->_mutex);

    if (this->_queue.empty()) {
      return false;
    }

    *data = this->_queue.front();
    this->_queue.pop();
    _condition_full.Signal();
    return true;
  }

  bool full() const {
    return this->_queue.size() >= _max_count;
  }

 private:
  CondVar _condition_full;
  const size_t _max_count;
  DISALLOW_COPY_AND_ASSIGN(FixedSizeConQueue);
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_LIB_BASE_CONCURRENT_QUEUE_H
