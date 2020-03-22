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

#ifndef CYBER_BASE_OBJECT_POOL_H_
#define CYBER_BASE_OBJECT_POOL_H_

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <new>
#include <utility>

#include "cyber/base/for_each.h"
#include "cyber/base/macros.h"

namespace apollo {
namespace cyber {
namespace base {

template <typename T>
class ObjectPool : public std::enable_shared_from_this<ObjectPool<T>> {
 public:
  using InitFunc = std::function<void(T *)>;
  using ObjectPoolPtr = std::shared_ptr<ObjectPool<T>>;

  template <typename... Args>
  explicit ObjectPool(uint32_t num_objects, Args &&... args);

  template <typename... Args>
  ObjectPool(uint32_t num_objects, InitFunc f, Args &&... args);

  virtual ~ObjectPool();

  std::shared_ptr<T> GetObject();

 private:
  struct Node {
    T object;
    Node *next;
  };

  ObjectPool(ObjectPool &) = delete;
  ObjectPool &operator=(ObjectPool &) = delete;
  void ReleaseObject(T *);

  uint32_t num_objects_ = 0;
  char *object_arena_ = nullptr;
  Node *free_head_ = nullptr;
};

template <typename T>
template <typename... Args>
ObjectPool<T>::ObjectPool(uint32_t num_objects, Args &&... args)
    : num_objects_(num_objects) {
  const size_t size = sizeof(Node);
  object_arena_ = static_cast<char *>(std::calloc(num_objects_, size));
  if (object_arena_ == nullptr) {
    throw std::bad_alloc();
  }

  FOR_EACH(i, 0, num_objects_) {
    T *obj = new (object_arena_ + i * size) T(std::forward<Args>(args)...);
    reinterpret_cast<Node *>(obj)->next = free_head_;
    free_head_ = reinterpret_cast<Node *>(obj);
  }
}

template <typename T>
template <typename... Args>
ObjectPool<T>::ObjectPool(uint32_t num_objects, InitFunc f, Args &&... args)
    : num_objects_(num_objects) {
  const size_t size = sizeof(Node);
  object_arena_ = static_cast<char *>(std::calloc(num_objects_, size));
  if (object_arena_ == nullptr) {
    throw std::bad_alloc();
  }

  FOR_EACH(i, 0, num_objects_) {
    T *obj = new (object_arena_ + i * size) T(std::forward<Args>(args)...);
    f(obj);
    reinterpret_cast<Node *>(obj)->next = free_head_;
    free_head_ = reinterpret_cast<Node *>(obj);
  }
}

template <typename T>
ObjectPool<T>::~ObjectPool() {
  if (object_arena_ != nullptr) {
    const size_t size = sizeof(Node);
    FOR_EACH(i, 0, num_objects_) {
      reinterpret_cast<Node *>(object_arena_ + i * size)->object.~T();
    }
    std::free(object_arena_);
  }
}

template <typename T>
void ObjectPool<T>::ReleaseObject(T *object) {
  if (cyber_unlikely(object == nullptr)) {
    return;
  }

  reinterpret_cast<Node *>(object)->next = free_head_;
  free_head_ = reinterpret_cast<Node *>(object);
}

template <typename T>
std::shared_ptr<T> ObjectPool<T>::GetObject() {
  if (cyber_unlikely(free_head_ == nullptr)) {
    return nullptr;
  }

  auto self = this->shared_from_this();
  auto obj =
      std::shared_ptr<T>(reinterpret_cast<T *>(free_head_),
                         [self](T *object) { self->ReleaseObject(object); });
  free_head_ = free_head_->next;
  return obj;
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_OBJECT_POOL_H_
