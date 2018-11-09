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

#ifndef CYBER_BASE_CONCURRENT_OBJECT_POOL_H_
#define CYBER_BASE_CONCURRENT_OBJECT_POOL_H_

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "cyber/base/for_each.h"
#include "cyber/base/macros.h"

namespace apollo {
namespace cyber {
namespace base {

template <typename T>
class CCObjectPool {
 private:
  struct Node {
    T object;
    Node *next;
  };
  struct Head {
    uintptr_t c;
    Node *node;
  };

  using CCObjectPoolPtr = std::shared_ptr<CCObjectPool<T>>;

 public:
  using InitFunc = std::function<void(T *)>;
  template <typename... Args>
  explicit CCObjectPool(uint32_t size, Args &&... args);

  template <typename... Args>
  CCObjectPool(uint32_t size, InitFunc f, Args &&... args);

  virtual ~CCObjectPool();

  std::shared_ptr<T> GetObject();
  void ReleaseObject(T *);
  void Dump() const;

 private:
  CCObjectPool(CCObjectPool &) = delete;
  CCObjectPool &operator=(CCObjectPool &) = delete;

  std::atomic<Head> pool_;
  Node *node_arena_ = nullptr;
  uint32_t num_object_ = 0;
};

template <typename T>
template <typename... Args>
CCObjectPool<T>::CCObjectPool(uint32_t size, Args &&... args)
    : num_object_(size) {
  node_arena_ = static_cast<Node *>(std::calloc(num_object_, sizeof(Node)));
  if (node_arena_ == nullptr) {
    throw std::bad_alloc();
  }

  char *m = reinterpret_cast<char *>(node_arena_);
  FOR_EACH(i, 0, num_object_) {
    new (m + i * sizeof(Node)) T(std::forward<Args>(args)...);
    node_arena_[i].next = node_arena_ + 1 + i;
  }
  node_arena_[num_object_ - 1].next = nullptr;

  pool_.store({0, node_arena_}, std::memory_order_relaxed);
}

template <typename T>
template <typename... Args>
CCObjectPool<T>::CCObjectPool(uint32_t size, InitFunc f, Args &&... args)
    : num_object_(size) {
  node_arena_ = static_cast<Node *>(std::calloc(num_object_, sizeof(Node)));
  if (node_arena_ == nullptr) {
    throw std::bad_alloc();
  }

  char *m = reinterpret_cast<char *>(node_arena_);
  FOR_EACH(i, 0, num_object_) {
    T *obj = new (m + i * sizeof(Node)) T(std::forward<Args>(args)...);
    f(obj);
    node_arena_[i].next = node_arena_ + 1 + i;
  }
  node_arena_[num_object_ - 1].next = nullptr;
  pool_.store({0, node_arena_}, std::memory_order_relaxed);
}

template <typename T>
CCObjectPool<T>::~CCObjectPool() {
  if (node_arena_ != nullptr) {
    FOR_EACH(i, 0, num_object_) { node_arena_[i].object.~T(); }
    std::free(node_arena_);
  }
}

template <typename T>
std::shared_ptr<T> CCObjectPool<T>::GetObject() {
  Head newh;
  Head oldh = pool_.load(std::memory_order_acquire);
  do {
    if (unlikely(oldh.node == nullptr)) return nullptr;

    newh.c = oldh.c + 1;
    newh.node = oldh.node->next;
  } while (!pool_.compare_exchange_weak(oldh, newh, std::memory_order_acq_rel,
                                        std::memory_order_acquire));

  return std::shared_ptr<T>(&(oldh.node->object),
                            [this](T *object) { this->ReleaseObject(object); });
}

template <typename T>
void CCObjectPool<T>::ReleaseObject(T *object) {
  Head newh;
  Node *n = reinterpret_cast<Node *>(object);

  Head oldh = pool_.load(std::memory_order_acquire);
  do {
    n->next = oldh.node;
    newh.c = oldh.c + 1;
    newh.node = n;
  } while (!pool_.compare_exchange_weak(oldh, newh, std::memory_order_acq_rel,
                                        std::memory_order_acquire));
}

template <typename T>
void CCObjectPool<T>::Dump() const {
  Head h = pool_.load(std::memory_order_acquire);
  Node *n = h.node;
  int c = 0;

  while (n) {
    c++;
    n = n->next;
  }
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_CONCURRENT_OBJECT_POOL_H_
