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
class CCObjectPool : public std::enable_shared_from_this<CCObjectPool<T>> {
 public:
  explicit CCObjectPool(uint32_t size);
  virtual ~CCObjectPool();

  template <typename... Args>
  void ConstructAll(Args &&... args);

  template <typename... Args>
  std::shared_ptr<T> ConstructObject(Args &&... args);

  std::shared_ptr<T> GetObject();
  void ReleaseObject(T *);
  uint32_t size() const;

 private:
  struct Node {
    T object;
    Node *next;
  };

  struct alignas(2 * sizeof(Node *)) Head {
    uintptr_t count;
    Node *node;
  };

 private:
  CCObjectPool(CCObjectPool &) = delete;
  CCObjectPool &operator=(CCObjectPool &) = delete;
  bool FindFreeHead(Head *head);

  std::atomic<Head> free_head_;
  Node *node_arena_ = nullptr;
  uint32_t capacity_ = 0;
};

template <typename T>
CCObjectPool<T>::CCObjectPool(uint32_t size) : capacity_(size) {
  node_arena_ = static_cast<Node *>(CheckedCalloc(capacity_, sizeof(Node)));
  FOR_EACH(i, 0, capacity_ - 1) { node_arena_[i].next = node_arena_ + 1 + i; }
  node_arena_[capacity_ - 1].next = nullptr;
  free_head_.store({0, node_arena_}, std::memory_order_relaxed);
}

template <typename T>
template <typename... Args>
void CCObjectPool<T>::ConstructAll(Args &&... args) {
  FOR_EACH(i, 0, capacity_) {
    new (node_arena_ + i) T(std::forward<Args>(args)...);
  }
}

template <typename T>
CCObjectPool<T>::~CCObjectPool() {
  std::free(node_arena_);
}

template <typename T>
bool CCObjectPool<T>::FindFreeHead(Head *head) {
  Head new_head;
  Head old_head = free_head_.load(std::memory_order_acquire);
  do {
    if (cyber_unlikely(old_head.node == nullptr)) {
      return false;
    }
    new_head.node = old_head.node->next;
    new_head.count = old_head.count + 1;
  } while (!free_head_.compare_exchange_weak(old_head, new_head,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire));
  *head = old_head;
  return true;
}

template <typename T>
std::shared_ptr<T> CCObjectPool<T>::GetObject() {
  Head free_head;
  if (cyber_unlikely(!FindFreeHead(&free_head))) {
    return nullptr;
  }
  auto self = this->shared_from_this();
  return std::shared_ptr<T>(reinterpret_cast<T *>(free_head.node),
                            [self](T *object) { self->ReleaseObject(object); });
}

template <typename T>
template <typename... Args>
std::shared_ptr<T> CCObjectPool<T>::ConstructObject(Args &&... args) {
  Head free_head;
  if (cyber_unlikely(!FindFreeHead(&free_head))) {
    return nullptr;
  }
  auto self = this->shared_from_this();
  T *ptr = new (free_head.node) T(std::forward<Args>(args)...);
  return std::shared_ptr<T>(ptr, [self](T *object) {
    object->~T();
    self->ReleaseObject(object);
  });
}

template <typename T>
void CCObjectPool<T>::ReleaseObject(T *object) {
  Head new_head;
  Node *node = reinterpret_cast<Node *>(object);
  Head old_head = free_head_.load(std::memory_order_acquire);
  do {
    node->next = old_head.node;
    new_head.node = node;
    new_head.count = old_head.count + 1;
  } while (!free_head_.compare_exchange_weak(old_head, new_head,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire));
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_CONCURRENT_OBJECT_POOL_H_
