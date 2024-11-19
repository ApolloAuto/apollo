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

#ifndef CYBER_BASE_ARENA_QUEUE_H_
#define CYBER_BASE_ARENA_QUEUE_H_

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <google/protobuf/arena.h>

#include "cyber/base/macros.h"
#include "cyber/base/wait_strategy.h"

namespace apollo {
namespace cyber {
namespace base {

template <typename T>
class ArenaQueue {
 public:
  using value_type = T;
  using size_type = uint64_t;

 public:
  ArenaQueue() {}
  ArenaQueue& operator=(const ArenaQueue& other) = delete;
  ArenaQueue(const ArenaQueue& other) = delete;
  ~ArenaQueue();
  bool Init(uint64_t size);
  bool Init(uint64_t size, google::protobuf::Arena* arena);

  T* AddBack();
  T* PopFront();
  T* GetBack();
  T* GetFront();

  uint64_t Size();
  bool Empty();
  uint64_t Head() { return head_.load(); }
  uint64_t Tail() { return tail_.load(); }
  uint64_t Commit() { return commit_.load(); }
  bool NextIndex(uint64_t& index) {
    if (Empty()) {
      return false;
    }
    if (arena_) {
      if (GetIndex(index) < Tail() - 1) {
        index = GetIndex(index + 1);
        return true;
      }
      return false;
    } else {
      if (index < Size() - 1) {
        index = index + 1;
        return true;
      }
      return false;
    }
  }
  bool GetHeadIndex(uint64_t& index) {
    if (Empty()) {
      return false;
    }
    if (arena_) {
      index = GetIndex(head_ + 1);
      return true;
    } else {
      index = 0;
      return true;
    }
  }
  bool GetTailIndex(uint64_t& index) {
    if (Empty()) {
      return false;
    }
    if (arena_) {
      index = GetIndex(tail_ - 1);
      return true;
    } else {
      index = Size() - 1;
      return true;
    }
  }
  bool GetEleByIndex(uint64_t i, T*& ptr) {
    if (Empty()) {
      return false;
    }
    if (arena_) {
      ptr = pool_[GetIndex(i)];
      return true;
    } else {
      if (i > Size() - 1) {
        return false;
      }
      ptr = &normal_queue[i];
      return true;
    }
  }
  bool IsArenaEnable() { return arena_; }

 private:
  uint64_t GetIndex(uint64_t num);

  alignas(CACHELINE_SIZE) std::atomic<uint64_t> head_ = {0};
  alignas(CACHELINE_SIZE) std::atomic<uint64_t> tail_ = {1};
  alignas(CACHELINE_SIZE) std::atomic<uint64_t> commit_ = {1};

  uint64_t pool_size_ = 0;
  std::vector<T*> pool_;
  bool arena_;
  std::deque<T> normal_queue;
};

template <typename T>
ArenaQueue<T>::~ArenaQueue() {}

template <typename T>
inline bool ArenaQueue<T>::Init(uint64_t size) {
  arena_ = false;
  return true;
}

template <typename T>
inline bool ArenaQueue<T>::Init(uint64_t size, google::protobuf::Arena* arena) {
  pool_size_ = size + 2;
  if (pool_.size() == pool_size_) {
    return true;
  }
  pool_.clear();
  for (uint64_t i = 0; i < pool_size_; ++i) {
    pool_.push_back(google::protobuf::Arena::CreateMessage<T>(arena));
  }
  arena_ = true;
  return true;
}

template <typename T>
T* ArenaQueue<T>::GetBack() {
  if (Empty()) {
    return nullptr;
  }
  if (arena_) {
    return pool_[GetIndex(tail_ - 1)];
  } else {
    return &normal_queue.back();
  }
}

template <typename T>
T* ArenaQueue<T>::GetFront() {
  if (Empty()) {
    return nullptr;
  }
  if (arena_) {
    return pool_[GetIndex(head_ + 1)];
  } else {
    return &normal_queue.front();
  }
}

template <typename T>
T* ArenaQueue<T>::AddBack() {
  if (arena_) {
    uint64_t new_tail = 0;
    uint64_t old_commit = 0;
    uint64_t old_tail = tail_.load(std::memory_order_acquire);
    do {
      new_tail = old_tail + 1;
      if (GetIndex(new_tail) ==
          GetIndex(head_.load(std::memory_order_acquire))) {
        return nullptr;
      }
    } while (!tail_.compare_exchange_weak(old_tail, new_tail,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed));
    do {
      old_commit = old_tail;
    } while (cyber_unlikely(!commit_.compare_exchange_weak(
        old_commit, new_tail, std::memory_order_acq_rel,
        std::memory_order_relaxed)));
    return pool_[GetIndex(old_tail)];
  } else {
    T instance;
    normal_queue.push_back(instance);
    return &normal_queue.back();
  }
}

template <typename T>
T* ArenaQueue<T>::PopFront() {
  if (Empty()) {
    return nullptr;
  }
  if (arena_) {
    uint64_t new_head = 0;
    uint64_t old_head = head_.load(std::memory_order_acquire);
    do {
      new_head = old_head + 1;
      if (new_head == commit_.load(std::memory_order_acquire)) {
        return nullptr;
      }
    } while (!head_.compare_exchange_weak(old_head, new_head,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed));
    return pool_[GetIndex(new_head)];
  } else {
    normal_queue.pop_front();
    return nullptr;
  }
}

template <typename T>
inline uint64_t ArenaQueue<T>::Size() {
  if (arena_) {
    return tail_ - head_ - 1;
  } else {
    return normal_queue.size();
  }
}

template <typename T>
inline bool ArenaQueue<T>::Empty() {
  if (arena_) {
    return Size() == 0;
  } else {
    return normal_queue.empty();
  }
}

template <typename T>
inline uint64_t ArenaQueue<T>::GetIndex(uint64_t num) {
  return num - (num / pool_size_) * pool_size_;  // faster than %
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_ARENA_QUEUE_H_
