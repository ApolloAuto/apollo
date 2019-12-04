/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file:
 **/

#pragma once

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

#include "modules/common/util/map_util.h"

namespace apollo {
namespace planning {

template <typename I, typename T>
class IndexedQueue {
 public:
  // Get infinite capacity with 0.
  explicit IndexedQueue(size_t capacity) : capacity_(capacity) {}

  const T *Find(const I id) const {
    auto *result = apollo::common::util::FindOrNull(map_, id);
    return result ? result->get() : nullptr;
  }

  const T *Latest() const {
    if (queue_.empty()) {
      return nullptr;
    }
    return Find(queue_.back().first);
  }

  bool Add(const I id, std::unique_ptr<T> ptr) {
    if (Find(id)) {
      return false;
    }
    if (capacity_ > 0 && queue_.size() == capacity_) {
      map_.erase(queue_.front().first);
      queue_.pop();
    }
    queue_.emplace(id, ptr.get());
    map_[id] = std::move(ptr);
    return true;
  }

  void Clear() {
    capacity_ = 0;
    while (!queue_.empty()) {
      queue_.pop();
    }
    map_.clear();
  }

 public:
  size_t capacity_ = 0;
  std::queue<std::pair<I, const T *>> queue_;
  std::unordered_map<I, std::unique_ptr<T>> map_;
};

}  // namespace planning
}  // namespace apollo
