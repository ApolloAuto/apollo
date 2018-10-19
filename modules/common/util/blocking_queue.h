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

#pragma once

#include <deque>
#include <string>
#include <vector>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/thread.hpp"

#include "cyber/common/log.h"

namespace apollo {
namespace common {
namespace util {

template <typename T>
class BlockingQueue {
 public:
  BlockingQueue(uint size, const std::string& name)
      : size_(size), name_(name) {}
  BlockingQueue() {}

  void init(uint size, const std::string& name) {
    size_ = size;
    name_ = name;
  }

  void put(const T x) {
    boost::mutex::scoped_lock lock(mutex_);
    if (size_ != 0 && queue_.size() > size_) {
      AINFO << "queue" << name_ << " is full, size: " << size_;
      queue_.pop_front();
    }
    queue_.push_back(x);
    not_empty_.notify_all();
  }

  T get(size_t index) {
    boost::mutex::scoped_lock lock(mutex_);
    while (queue_.empty()) {
      not_empty_.wait(lock);
    }
    return queue_.at(index);
  }

  bool take(T* x, int timeout_sec) {
    boost::mutex::scoped_lock lock(mutex_);
    bool recv_data = false;
    while (queue_.empty()) {
      recv_data = not_empty_.timed_wait(
          lock,
          boost::get_system_time() + boost::posix_time::seconds(timeout_sec));
      if (!recv_data) {
        return false;
      }
    }

    *x = queue_.front();
    queue_.pop_front();
    return true;
  }

  T take() {
    boost::mutex::scoped_lock lock(mutex_);
    while (queue_.empty()) {
      not_empty_.wait(lock);
    }
    const T front = queue_.front();
    queue_.pop_front();
    return front;
  }

  int take(std::vector<T>* v, size_t count) {
    boost::mutex::scoped_lock lock(mutex_);
    while (queue_.empty()) {
      not_empty_.wait(lock);
    }
    while (!queue_.empty() && count--) {
      v->push_back(queue_.front());
      queue_.pop_front();
    }

    return v->size();
  }

  size_t size() const {
    boost::mutex::scoped_lock lock(mutex_);
    return queue_.size();
  }

 private:
  mutable boost::mutex mutex_;
  boost::condition_variable not_empty_;
  std::deque<T> queue_;
  uint size_ = 0;
  std::string name_;
};

}  // namespace util
}  // namespace common
}  // namespace apollo
