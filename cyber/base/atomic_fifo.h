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

#ifndef CYBER_BASE_ATOMIC_FIFO_H_
#define CYBER_BASE_ATOMIC_FIFO_H_

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "cyber/base/macros.h"

namespace apollo {
namespace cyber {

template <typename T>
class AtomicFIFO {
 private:
  struct Node {
    T value;
  };

 public:
  static AtomicFIFO *GetInstance(int cap = 100) {
    static AtomicFIFO *inst = new AtomicFIFO(cap);
    return inst;
  }

  bool Push(const T &value);
  bool Pop(T *value);
  // insert();

 private:
  Node *node_arena_;
  alignas(CACHELINE_SIZE) std::atomic<uint32_t> head_;
  alignas(CACHELINE_SIZE) std::atomic<uint32_t> commit_;
  alignas(CACHELINE_SIZE) std::atomic<uint32_t> tail_;
  int capacity_;

  explicit AtomicFIFO(int cap);
  ~AtomicFIFO();
  AtomicFIFO(AtomicFIFO &) = delete;
  AtomicFIFO &operator=(AtomicFIFO &) = delete;
};

template <typename T>
AtomicFIFO<T>::AtomicFIFO(int cap) : capacity_(cap) {
  node_arena_ = static_cast<Node *>(malloc(capacity_ * sizeof(Node)));
  memset(node_arena_, 0, capacity_ * sizeof(Node));

  head_.store(0, std::memory_order_relaxed);
  tail_.store(0, std::memory_order_relaxed);
  commit_.store(0, std::memory_order_relaxed);
}

template <typename T>
AtomicFIFO<T>::~AtomicFIFO() {
  if (node_arena_ != nullptr) {
    for (int i = 0; i < capacity_; i++) {
      node_arena_[i].value.~T();
    }
    free(node_arena_);
  }
}

template <typename T>
bool AtomicFIFO<T>::Push(const T &value) {
  uint32_t oldt, newt;

  oldt = tail_.load(std::memory_order_acquire);
  do {
    uint32_t h = head_.load(std::memory_order_acquire);
    uint32_t t = tail_.load(std::memory_order_acquire);

    if (((t + 1) % capacity_) == h) return false;

    newt = (oldt + 1) % capacity_;
  } while (!tail_.compare_exchange_weak(oldt, newt, std::memory_order_acq_rel,
                                        std::memory_order_acquire));

  (node_arena_ + oldt)->value = value;

  while (unlikely(commit_.load(std::memory_order_acquire) != oldt)) cpu_relax();

  commit_.store(newt, std::memory_order_release);

  return true;
}

template <typename T>
bool AtomicFIFO<T>::Pop(T *value) {
  uint32_t oldh, newh;

  oldh = head_.load(std::memory_order_acquire);

  do {
    uint32_t h = head_.load(std::memory_order_acquire);
    uint32_t c = commit_.load(std::memory_order_acquire);

    if (h == c) return false;

    newh = (oldh + 1) % capacity_;

    *value = (node_arena_ + oldh)->value;
  } while (!head_.compare_exchange_weak(oldh, newh, std::memory_order_acq_rel,
                                        std::memory_order_acquire));

  return true;
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_ATOMIC_FIFO_H_
