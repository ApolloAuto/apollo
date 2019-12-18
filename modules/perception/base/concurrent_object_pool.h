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
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include "modules/perception/base/object_pool.h"

#define PERCEPTION_BASE_DISABLE_POOL
namespace apollo {
namespace perception {
namespace base {

static const size_t kPoolDefaultExtendNum = 10;
static const size_t kPoolDefaultSize = 100;

// @brief default initializer used in concurrent object pool
template <class T>
struct ObjectPoolDefaultInitializer {
  void operator()(T* t) const {}
};
// @brief concurrent object pool with dynamic size
template <class ObjectType, size_t N = kPoolDefaultSize,
          class Initializer = ObjectPoolDefaultInitializer<ObjectType>>
class ConcurrentObjectPool : public BaseObjectPool<ObjectType> {
 public:
  // using ObjectTypePtr = typename BaseObjectPool<ObjectType>::ObjectTypePtr;
  using BaseObjectPool<ObjectType>::capacity_;
  // @brief Only allow accessing from global instance
  static ConcurrentObjectPool& Instance() {
    static ConcurrentObjectPool pool(N);
    return pool;
  }
  // @brief overrided function to get object smart pointer
  std::shared_ptr<ObjectType> Get() override {
// TODO(All): remove conditional build
#ifndef PERCEPTION_BASE_DISABLE_POOL
    ObjectType* ptr = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.empty()) {
        Add(1 + kPoolDefaultExtendNum);
      }
      ptr = queue_.front();
      queue_.pop();
    }
    // For efficiency consideration, initialization should be invoked
    // after releasing the mutex
    kInitializer(ptr);
    return std::shared_ptr<ObjectType>(ptr, [&](ObjectType* obj_ptr) {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push(obj_ptr);
    });
#else
    return std::shared_ptr<ObjectType>(new ObjectType);
#endif
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[OUT] data: vector container to store the pointers
  void BatchGet(size_t num,
                std::vector<std::shared_ptr<ObjectType>>* data) override {
#ifndef PERCEPTION_BASE_DISABLE_POOL
    std::vector<ObjectType*> buffer(num, nullptr);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() < num) {
        Add(num - queue_.size() + kPoolDefaultExtendNum);
      }
      for (size_t i = 0; i < num; ++i) {
        buffer[i] = queue_.front();
        queue_.pop();
      }
    }
    // For efficiency consideration, initialization should be invoked
    // after releasing the mutex
    for (size_t i = 0; i < num; ++i) {
      kInitializer(buffer[i]);
      data->emplace_back(
          std::shared_ptr<ObjectType>(buffer[i], [&](ObjectType* obj_ptr) {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(obj_ptr);
          }));
    }
#else
    for (size_t i = 0; i < num; ++i) {
      data->emplace_back(std::shared_ptr<ObjectType>(new ObjectType));
    }
#endif
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the list
  // @params[OUT] data: list container to store the pointers
  void BatchGet(size_t num, bool is_front,
                std::list<std::shared_ptr<ObjectType>>* data) override {
#ifndef PERCEPTION_BASE_DISABLE_POOL
    std::vector<ObjectType*> buffer(num, nullptr);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() < num) {
        Add(num - queue_.size() + kPoolDefaultExtendNum);
      }
      for (size_t i = 0; i < num; ++i) {
        buffer[i] = queue_.front();
        queue_.pop();
      }
    }
    // For efficiency consideration, initialization should be invoked
    // after releasing the mutex
    for (size_t i = 0; i < num; ++i) {
      kInitializer(buffer[i]);
      is_front ? data->emplace_front(std::shared_ptr<ObjectType>(
                     buffer[i],
                     [&](ObjectType* obj_ptr) {
                       std::lock_guard<std::mutex> lock(mutex_);
                       queue_.push(obj_ptr);
                     }))
               : data->emplace_back(std::shared_ptr<ObjectType>(
                     buffer[i], [&](ObjectType* obj_ptr) {
                       std::lock_guard<std::mutex> lock(mutex_);
                       queue_.push(obj_ptr);
                     }));
    }
#else
    for (size_t i = 0; i < num; ++i) {
      is_front
          ? data->emplace_front(std::shared_ptr<ObjectType>(new ObjectType))
          : data->emplace_back(std::shared_ptr<ObjectType>(new ObjectType));
    }
#endif
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the deque
  // @params[OUT] data: deque container to store the pointers
  void BatchGet(size_t num, bool is_front,
                std::deque<std::shared_ptr<ObjectType>>* data) override {
#ifndef PERCEPTION_BASE_DISABLE_POOL
    std::vector<ObjectType*> buffer(num, nullptr);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() < num) {
        Add(num - queue_.size() + kPoolDefaultExtendNum);
      }
      for (size_t i = 0; i < num; ++i) {
        buffer[i] = queue_.front();
        queue_.pop();
      }
    }
    for (size_t i = 0; i < num; ++i) {
      kInitializer(buffer[i]);
      is_front ? data->emplace_front(std::shared_ptr<ObjectType>(
                     buffer[i],
                     [&](ObjectType* obj_ptr) {
                       std::lock_guard<std::mutex> lock(mutex_);
                       queue_.push(obj_ptr);
                     }))
               : data->emplace_back(std::shared_ptr<ObjectType>(
                     buffer[i], [&](ObjectType* obj_ptr) {
                       std::lock_guard<std::mutex> lock(mutex_);
                       queue_.push(obj_ptr);
                     }));
    }
#else
    for (size_t i = 0; i < num; ++i) {
      is_front
          ? data->emplace_front(std::shared_ptr<ObjectType>(new ObjectType))
          : data->emplace_back(std::shared_ptr<ObjectType>(new ObjectType));
    }
#endif
  }
#ifndef PERCEPTION_BASE_DISABLE_POOL
  // @brief overrided function to set capacity
  void set_capacity(size_t capacity) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (capacity_ < capacity) {
      Add(capacity - capacity_);
    }
  }
  // @brief get remained object number
  size_t RemainedNum() override { return queue_.size(); }
#endif
  // @brief destructor to release the cached memory
  ~ConcurrentObjectPool() override {
    if (cache_) {
      delete[] cache_;
      cache_ = nullptr;
    }
    for (auto& ptr : extended_cache_) {
      delete ptr;
    }
    extended_cache_.clear();
  }

 protected:
// @brief add num objects, should add lock before invoke this function
#ifndef PERCEPTION_BASE_DISABLE_POOL
  void Add(size_t num) {
    for (size_t i = 0; i < num; ++i) {
      ObjectType* ptr = new ObjectType;
      extended_cache_.push_back(ptr);
      queue_.push(ptr);
    }
    capacity_ = kDefaultCacheSize + extended_cache_.size();
  }
#endif
  // @brief default constructor
  explicit ConcurrentObjectPool(const size_t default_size)
      : kDefaultCacheSize(default_size) {
#ifndef PERCEPTION_BASE_DISABLE_POOL
    cache_ = new ObjectType[kDefaultCacheSize];
    for (size_t i = 0; i < kDefaultCacheSize; ++i) {
      queue_.push(&cache_[i]);
    }
    capacity_ = kDefaultCacheSize;
#endif
  }
  std::mutex mutex_;
  std::queue<ObjectType*> queue_;
  // @brief point to a continuous memory of default pool size
  ObjectType* cache_ = nullptr;
  const size_t kDefaultCacheSize;
  // @brief list to store extended memory, not as efficient
  std::list<ObjectType*> extended_cache_;
  static const Initializer kInitializer;
};

}  // namespace base
}  // namespace perception
}  // namespace apollo
