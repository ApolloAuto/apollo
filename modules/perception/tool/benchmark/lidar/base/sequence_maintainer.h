/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <iostream>
#include <list>
#include <map>
#include <vector>
#include "modules/perception/tool/benchmark/lidar/util/object.h"

namespace apollo {
namespace perception {
namespace benchmark {

template <typename ObjectKey>
using SequenceType = std::map<ObjectKey, ObjectPtr>;

template <typename ObjectKey>
class SequenceMaintainer {
 public:
  SequenceMaintainer() = default;
  ~SequenceMaintainer() = default;
  bool add_data(const std::vector<ObjectPtr>& objects, ObjectKey key);
  auto get_sequence(int sequence_id) -> SequenceType<ObjectKey>*;
  void clear() { _sequence.clear(); }

 protected:
  std::map<int, SequenceType<ObjectKey>> _sequence;

 private:
  static const std::size_t _s_max_sample_num = 10;
  static constexpr double _s_max_lift_time = 10.0;
};

template <typename ObjectKey>
bool SequenceMaintainer<ObjectKey>::add_data(
    const std::vector<ObjectPtr>& objects, ObjectKey key) {
  for (const auto& obj : objects) {
    auto& id = obj->track_id;
    if (_sequence.size() > 0 && _sequence.begin()->first > id) {
      std::cerr << "Find track_id roll back, so clear the cache sequence, "
                << "current id " << id << " oldest id "
                << _sequence.begin()->first << "." << std::endl;
      _sequence.clear();
    }
    auto& sub = _sequence[id];
    if (sub.size() > 0 && sub.rbegin()->first >= key) {
      // std::cerr << "New added key can not be less than old key, "
      //    << key << " to be added but " << sub.rbegin()->first << " exist."
      //    << std::endl;
      return false;
    }
    sub[key] = obj;
    auto iter = sub.begin();
    while (sub.size() > _s_max_sample_num) {
      sub.erase(iter++);
    }
  }
  auto iter = _sequence.begin();
  while (iter != _sequence.end()) {
    if (iter->second.empty() ||
        static_cast<double>(key - iter->second.rbegin()->first) >
            _s_max_lift_time) {
      _sequence.erase(iter++);
    } else {
      ++iter;
    }
  }
  return true;
}

template <typename ObjectKey>
auto SequenceMaintainer<ObjectKey>::get_sequence(int sequence_id)
    -> SequenceType<ObjectKey>* {
  auto iter = _sequence.find(sequence_id);
  if (iter == _sequence.end()) {
    return nullptr;
  } else {
    return &(iter->second);
  }
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
