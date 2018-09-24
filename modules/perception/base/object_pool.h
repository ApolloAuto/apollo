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
#ifndef PERCEPTION_BASE_OBJECT_POOL_H_
#define PERCEPTION_BASE_OBJECT_POOL_H_
#include <deque>
#include <list>
#include <memory>
#include <vector>

namespace apollo {
namespace perception {
namespace base {
// @brief general object pool interface
template <class ObjectType>
class BaseObjectPool {
 public:
  typedef std::shared_ptr<ObjectType> ObjectTypePtr;
  // @brief default constructor
  BaseObjectPool() = default;
  // @brief default destructor
  virtual ~BaseObjectPool() = default;
  // @brief pure virtual function to get object smart pointer
  virtual ObjectTypePtr Get() = 0;
  // @brief pure virtual function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[OUT] data: vector container to store the pointers
  virtual void BatchGet(size_t num, std::vector<ObjectTypePtr>* data) = 0;
  // @brief pure virtual function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the list
  // @params[OUT] data: list container to store the pointers
  virtual void BatchGet(size_t num, bool is_front,
                        std::list<ObjectTypePtr>* data) = 0;
  // @brief pure virtual function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the deque
  // @params[OUT] data: deque container to store the pointers
  virtual void BatchGet(size_t num, bool is_front,
                        std::deque<ObjectTypePtr>* data) = 0;
  // @brief virtual function to set capacity
  virtual void set_capacity(size_t capacity) {}
  // @brief capacity getter
  size_t get_capacity() { return capacity_; }
  // @brief get remained object number
  virtual size_t RemainedNum() { return 0; }

 protected:
  BaseObjectPool(const BaseObjectPool& rhs) = delete;
  BaseObjectPool& operator=(const BaseObjectPool& rhs) = delete;
  size_t capacity_ = 0;
};  // class BaseObjectPool

// @brief dummy object pool implementation, not managing memory
template <class ObjectType>
class DummyObjectPool : public BaseObjectPool<ObjectType> {
 public:
  using typename BaseObjectPool<ObjectType>::ObjectTypePtr;
  // @brief Only allow accessing from global instance
  static DummyObjectPool& Instance() {
    static DummyObjectPool pool;
    return pool;
  }
  // @brief overrided function to get object smart pointer
  ObjectTypePtr Get() override {
    return std::shared_ptr<ObjectType>(new ObjectType);
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[OUT] data: vector container to store the pointers
  void BatchGet(size_t num, std::vector<ObjectTypePtr>* data) override {
    for (size_t i = 0; i < num; ++i) {
      data->emplace_back(ObjectTypePtr(new ObjectType));
    }
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the list
  // @params[OUT] data: list container to store the pointers
  void BatchGet(size_t num, bool is_front,
                std::list<ObjectTypePtr>* data) override {
    for (size_t i = 0; i < num; ++i) {
      is_front ? data->emplace_front(ObjectTypePtr(new ObjectType))
               : data->emplace_back(ObjectTypePtr(new ObjectType));
    }
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the deque
  // @params[OUT] data: deque container to store the pointers
  void BatchGet(size_t num, bool is_front,
                std::deque<ObjectTypePtr>* data) override {
    for (size_t i = 0; i < num; ++i) {
      is_front ? data->emplace_front(ObjectTypePtr(new ObjectType))
               : data->emplace_back(ObjectTypePtr(new ObjectType));
    }
  }

 protected:
  // @brief default constructor
  DummyObjectPool() = default;
};  // class DummyObjectPool

}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_BASE_OBJECT_POOL_H_
