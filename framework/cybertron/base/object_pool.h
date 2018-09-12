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

#ifndef CYBERTRON_BASE_OBJECT_POOL_H_
#define CYBERTRON_BASE_OBJECT_POOL_H_

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <utility>

#include "cybertron/base/macros.h"

namespace apollo {
namespace cybertron {
namespace base {

template <typename T>
class ObjectPool {
 private:
  struct Node {
    T object;
    Node *next;
  };

  using ObjectPoolPtr = std::shared_ptr<ObjectPool<T>>;
  void ReleaseObject(T *);

 public:
  using InitFunc = std::function<void(T *)>;
  template <typename... Args>
  static ObjectPoolPtr Instance(int num_objects, Args &&... args) {
    static ObjectPoolPtr inst = std::shared_ptr<ObjectPool<T>>(
        new ObjectPool<T>(num_objects, std::forward<Args>(args)...));
    return inst;
  }

  template <typename... Args>
  static ObjectPoolPtr Instance(int num_objects, InitFunc &&f,
                                Args &&... args) {
    static ObjectPoolPtr inst = std::shared_ptr<ObjectPool<T>>(
        new ObjectPool<T>(num_objects, std::forward<InitFunc>(f),
                          std::forward<Args>(args)...));
    return inst;
  }

  ~ObjectPool();
  std::shared_ptr<T> GetObject();

  void Dump();

 private:
  template <typename... Args>
  explicit ObjectPool(int num_objects, Args &&... args);

  template <typename... Args>
  explicit ObjectPool(int num_objects, InitFunc f, Args &&... args);

  ObjectPool(ObjectPool &) = delete;
  ObjectPool &operator=(ObjectPool &) = delete;

  int num_objects_;
  char *object_arena_;
  Node *free_head_ = nullptr;
};

template <typename T>
template <typename... Args>
ObjectPool<T>::ObjectPool(int num_objects, Args &&... args)
    : num_objects_(num_objects) {
  int size = sizeof(Node);
  object_arena_ = static_cast<char *>(malloc(num_objects_ * size));
  memset(object_arena_, 0, num_objects_ * size);

  for (int i = 0; i < num_objects_; i++) {
    T *obj = new (object_arena_ + i * size) T(std::forward<Args>(args)...);
    reinterpret_cast<Node *>(obj)->next = free_head_;
    free_head_ = reinterpret_cast<Node *>(obj);
  }
}

template <typename T>
template <typename... Args>
ObjectPool<T>::ObjectPool(int num_objects, InitFunc f, Args &&... args)
    : num_objects_(num_objects) {
  int size = sizeof(Node);
  object_arena_ = static_cast<char *>(malloc(num_objects_ * size));
  memset(object_arena_, 0, num_objects_ * size);

  for (int i = 0; i < num_objects_; i++) {
    T *obj = new (object_arena_ + i * size) T(std::forward<Args>(args)...);
    f(obj);

    reinterpret_cast<Node *>(obj)->next = free_head_;
    free_head_ = reinterpret_cast<Node *>(obj);
  }
}

template <typename T>
ObjectPool<T>::~ObjectPool() {
  if (object_arena_ != nullptr) {
    int size = sizeof(Node);
    for (int i = 0; i < num_objects_; i++) {
      reinterpret_cast<Node *>(object_arena_ + i * size)->object.~T();
    }
    free(object_arena_);
  }
}

template <typename T>
void ObjectPool<T>::ReleaseObject(T *object) {
  if (unlikely(object == nullptr)) return;

  reinterpret_cast<Node *>(object)->next = free_head_;
  free_head_ = reinterpret_cast<Node *>(object);
}

template <typename T>
std::shared_ptr<T> ObjectPool<T>::GetObject() {
  if (unlikely(free_head_ == nullptr)) return nullptr;

  auto obj =
      std::shared_ptr<T>(reinterpret_cast<T *>(free_head_),
                         [this](T *object) { this->ReleaseObject(object); });
  free_head_ = free_head_->next;
  return obj;
}

template <typename T>
void ObjectPool<T>::Dump() {
  Node *n = free_head_;
  while (n) {
    n = n->next;
  }
}

}  // namespace base
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_BASE_OBJECT_POOL_H_
