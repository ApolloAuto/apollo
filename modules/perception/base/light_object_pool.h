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
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/base/concurrent_object_pool.h"
#include "modules/perception/base/sensor_meta.h"

namespace apollo {
namespace perception {
namespace base {

// @brief a simplified light object pool from concurrent object pool, can be
//        specialized for different sensors
template <class ObjectType, size_t N = kPoolDefaultSize,
          class Initializer = ObjectPoolDefaultInitializer<ObjectType>,
          SensorType sensor_type = SensorType::UNKNOWN_SENSOR_TYPE>
class LightObjectPool : public BaseObjectPool<ObjectType> {
 public:
  using BaseObjectPool<ObjectType>::capacity_;

  // @brief Only allow accessing from global instance
  static LightObjectPool& Instance(
      const std::string& sensor_name = "velodyne64") {
    typedef std::shared_ptr<LightObjectPool> LightObjectPoolPtr;
    static std::map<std::string, LightObjectPoolPtr> object_pool;
    auto itr = object_pool.find(sensor_name);
    if (itr == object_pool.end()) {
      auto ret = object_pool.insert(std::pair<std::string, LightObjectPoolPtr>(
          sensor_name, LightObjectPoolPtr(new LightObjectPool(N))));
      return *(ret.first->second);
    }
    return *(itr->second);
  }

  // @brief overrided function to get object smart pointer
  std::shared_ptr<ObjectType> Get() override {
    ObjectType* ptr = nullptr;
    if (queue_.empty()) {
      Add(1 + kPoolDefaultExtendNum);
    }
    ptr = queue_.front();
    queue_.pop();
    kInitializer(ptr);
    return std::shared_ptr<ObjectType>(
        ptr, [&](ObjectType* obj_ptr) { queue_.push(obj_ptr); });
  }

  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[OUT] data: vector container to store the pointers
  void BatchGet(size_t num,
                std::vector<std::shared_ptr<ObjectType>>* data) override {
    if (queue_.size() < num) {
      Add(num - queue_.size() + kPoolDefaultExtendNum);
    }
    ObjectType* ptr = nullptr;
    for (size_t i = 0; i < num; ++i) {
      ptr = queue_.front();
      queue_.pop();
      kInitializer(ptr);
      data->emplace_back(std::shared_ptr<ObjectType>(
          ptr, [&](ObjectType* obj_ptr) { queue_.push(obj_ptr); }));
    }
  }

  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the list
  // @params[OUT] data: list container to store the pointers
  void BatchGet(size_t num, bool is_front,
                std::list<std::shared_ptr<ObjectType>>* data) override {
    std::vector<ObjectType*> buffer(num, nullptr);
    if (queue_.size() < num) {
      Add(num - queue_.size() + kPoolDefaultExtendNum);
    }
    for (size_t i = 0; i < num; ++i) {
      buffer[i] = queue_.front();
      queue_.pop();
    }
    for (size_t i = 0; i < num; ++i) {
      kInitializer(buffer[i]);
      is_front
          ? data->emplace_front(std::shared_ptr<ObjectType>(
                buffer[i], [&](ObjectType* obj_ptr) { queue_.push(obj_ptr); }))
          : data->emplace_back(std::shared_ptr<ObjectType>(
                buffer[i], [&](ObjectType* obj_ptr) { queue_.push(obj_ptr); }));
    }
  }

  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the deque
  // @params[OUT] data: deque container to store the pointers
  void BatchGet(size_t num, bool is_front,
                std::deque<std::shared_ptr<ObjectType>>* data) override {
    std::vector<ObjectType*> buffer(num, nullptr);
    if (queue_.size() < num) {
      Add(num - queue_.size() + kPoolDefaultExtendNum);
    }
    for (size_t i = 0; i < num; ++i) {
      buffer[i] = queue_.front();
      queue_.pop();
    }
    for (size_t i = 0; i < num; ++i) {
      kInitializer(buffer[i]);
      is_front
          ? data->emplace_front(std::shared_ptr<ObjectType>(
                buffer[i], [&](ObjectType* obj_ptr) { queue_.push(obj_ptr); }))
          : data->emplace_back(std::shared_ptr<ObjectType>(
                buffer[i], [&](ObjectType* obj_ptr) { queue_.push(obj_ptr); }));
    }
  }

  // @brief overrided function to set capacity
  void set_capacity(size_t capacity) override {
    if (capacity_ < capacity) {
      Add(capacity - capacity_);
    }
  }

  // @brief get remained object number
  size_t RemainedNum() override { return queue_.size(); }
  // @brief destructor to release the cached memory
  ~LightObjectPool() override {
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
  void Add(size_t num) {
    for (size_t i = 0; i < num; ++i) {
      ObjectType* ptr = new ObjectType;
      extended_cache_.push_back(ptr);
      queue_.push(ptr);
    }
    capacity_ = kDefaultCacheSize + extended_cache_.size();
  }

  // @brief default constructor
  explicit LightObjectPool(const size_t default_size)
      : kDefaultCacheSize(default_size) {
    cache_ = new ObjectType[kDefaultCacheSize];
    for (size_t i = 0; i < kDefaultCacheSize; ++i) {
      queue_.push(&cache_[i]);
    }
    capacity_ = kDefaultCacheSize;
  }

  std::queue<ObjectType*> queue_;
  // @brief point to a continuous memory of default pool size
  ObjectType* cache_ = nullptr;
  const size_t kDefaultCacheSize;
  // @brief list to store extended memory, not as efficient
  std::list<ObjectType*> extended_cache_;
  // TODO(All): Fix lint issue with static const
  Initializer kInitializer;
  // Initializer kInitializer;
};

}  // namespace base
}  // namespace perception
}  // namespace apollo
