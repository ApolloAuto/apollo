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
 * @file indexed_list.h
 **/

#ifndef MODULES_PLANNING_COMMON_INDEXED_LIST_H_
#define MODULES_PLANNING_COMMON_INDEXED_LIST_H_

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace apollo {
namespace planning {

template <typename I, typename T>
class IndexedList {
 public:
  bool Add(const I id, std::unique_ptr<T> ptr) {
    if (Find(id)) {
      return false;
    }
    _object_list.push_back(ptr.get());
    _object_dict[id] = std::move(ptr);
    return true;
  }
  T* Find(const I id) {
    auto iter = _object_dict.find(id);
    if (iter == _object_dict.end()) {
      return nullptr;
    } else {
      return iter->second.get();
    }
  }
  const std::vector<const T*>& Items() const { return _object_list; }

 private:
  std::vector<const T*> _object_list;
  std::unordered_map<I, std::unique_ptr<T>> _object_dict;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_INDEXED_LIST_H
