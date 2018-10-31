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

#ifndef CYBER_BASE_BOUNDED_QUEUE_H_
#define CYBER_BASE_BOUNDED_QUEUE_H_

#include <stdint.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <memory>

#include "cyber/base/macros.h"
#include "cyber/base/wait_strategy.h"

namespace apollo {
namespace cyber {
namespace base {

template <typename T>
class BoundedQueue {
 public:
  using value_type = T;
  using size_type = uint64_t;
  // avoid false sharing
  struct AtomicBool {
    alignas(CACHELINE_SIZE) volatile bool flag;
  };

 public:
  BoundedQueue() {}
  BoundedQueue& operator=(const BoundedQueue& other) = delete;
  BoundedQueue(const BoundedQueue& other) = delete;
  ~BoundedQueue();
  bool Init(uint64_t size);
  bool Init(uint64_t size, WaitStrategy* strategy);
  bool Enqueue(const T& element);
  bool Dequeue(T* element);
  bool WaitDequeue(T* element);
  uint64_t Size();
  bool Empty();
  void SetWaitStrategy(WaitStrategy* WaitStrategy);
  void BreakAllWait();

 private:
  uint64_t GetIndex(uint64_t num);

  alignas(CACHELINE_SIZE) std::atomic<uint64_t> head_ = {0};
  alignas(CACHELINE_SIZE) std::atomic<uint64_t> tail_ = {1};
  // alignas(CACHELINE_SIZE) std::atomic<uint64_t> size_ = {0};
  uint64_t pool_size_ = 0;
  T* pool_ = nullptr;
  AtomicBool* flags_ = nullptr;
  std::unique_ptr<WaitStrategy> wait_strategy_ = nullptr;
  volatile bool break_all_wait_ = false;
};

template <typename T>
BoundedQueue<T>::~BoundedQueue() {
  if (wait_strategy_) {
    BreakAllWait();
  }
  if (pool_) {
    for (int i = 0; i < pool_size_; ++i) {
      pool_[i].~T();
    }
    std::free(pool_);
  }
  std::free(flags_);
}

template <typename T>
bool BoundedQueue<T>::Init(uint64_t size) {
  return Init(size, new SleepWaitStrategy());
}

template <typename T>
bool BoundedQueue<T>::Init(uint64_t size, WaitStrategy* strategy) {
  // Head and tail each occupy a space
  pool_size_ = size + 2;
  pool_ = reinterpret_cast<T*>(std::calloc(pool_size_, sizeof(T)));
  if (pool_ == nullptr) {
    return false;
  }
  for (int i = 0; i < pool_size_; ++i) {
    new (&(pool_[i])) T();
  }
  flags_ = reinterpret_cast<AtomicBool*>(
      std::calloc(pool_size_, sizeof(AtomicBool)));
  if (flags_ == nullptr) {
    return false;
  }
  for (int i = 0; i < pool_size_; ++i) {
    flags_[i].flag = false;
  }
  wait_strategy_.reset(strategy);
  return true;
}

template <typename T>
bool BoundedQueue<T>::Enqueue(const T& element) {
  uint64_t new_tail = 0;
  uint64_t old_tail = tail_.load(std::memory_order_acquire);
  do {
    new_tail = GetIndex(old_tail + 1);
    if (new_tail == head_.load(std::memory_order_acquire)) {
      return false;
    }
  } while (!tail_.compare_exchange_weak(old_tail, new_tail,
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  // make sure the value of pool_[old_tail] has been read
  while (flags_[old_tail].flag) {
    cpu_relax();
  }
  pool_[old_tail] = element;
  flags_[old_tail].flag = true;
  wait_strategy_->NotifyOne();
  return true;
}

template <typename T>
bool BoundedQueue<T>::Dequeue(T* element) {
  uint64_t new_head = 0;
  uint64_t old_head = head_.load(std::memory_order_acquire);
  do {
    new_head = GetIndex(old_head + 1);
    if (new_head == tail_.load(std::memory_order_acquire)) {
      return false;
    }
  } while (!head_.compare_exchange_weak(old_head, new_head,
                                        std::memory_order_acq_rel,
                                        std::memory_order_relaxed));
  // make sure the value of pool_[new_head] has been written
  while (!flags_[new_head].flag) {
    cpu_relax();
  }
  *element = pool_[new_head];
  flags_[new_head].flag = false;
  return true;
}

template <typename T>
bool BoundedQueue<T>::WaitDequeue(T* element) {
  while (!break_all_wait_) {
    bool wait = false;
    uint64_t new_head = 0;
    uint64_t old_head = head_.load(std::memory_order_acquire);
    do {
      new_head = GetIndex(old_head + 1);
      if (new_head == tail_.load(std::memory_order_acquire)) {
        wait = true;
        break;
      }
    } while (!head_.compare_exchange_weak(old_head, new_head,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed));
    if (wait) {
      if (!wait_strategy_->EmptyWait()) {
        // wait timeout
        return false;
      }
      continue;
    }
    while (!flags_[new_head].flag) {
      cpu_relax();
    }
    *element = pool_[new_head];
    flags_[new_head].flag = false;
    return true;
  }
  return false;
}

template <typename T>
uint64_t BoundedQueue<T>::Size() {
  if (head_ >= tail_) {
    return tail_ + pool_size_ - head_ - 1;
  } else {
    return tail_ - head_ - 1;
  }
}

template <typename T>
bool BoundedQueue<T>::Empty() {
  return Size() == 0;
}

// In most cases num is less than pool_size_, this implementation is faster than
// %.
template <typename T>
uint64_t BoundedQueue<T>::GetIndex(uint64_t num) {
  while (num >= pool_size_) {
    num -= pool_size_;
  }
  return num;
}

template <typename T>
void BoundedQueue<T>::SetWaitStrategy(WaitStrategy* strategy) {
  wait_strategy_.reset(strategy);
}

template <typename T>
void BoundedQueue<T>::BreakAllWait() {
  break_all_wait_ = true;
  wait_strategy_->BreakAllWait();
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_BOUNDED_QUEUE_H_
