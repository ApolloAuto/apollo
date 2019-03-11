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

/**
 * @file tm_list.h
 * @brief The class of TimeMarkedList.
 */

#ifndef MODULES_LOCALIZATION_LMD_COMMON_TM_LIST_H_
#define MODULES_LOCALIZATION_LMD_COMMON_TM_LIST_H_

#include <algorithm>
#include <map>
#include <utility>

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class TimeMarkedList
 *
 * @brief  A time marked list.
 */
template <class Data>
class TimeMarkedList : public std::map<double, Data> {
 public:
  typedef typename std::map<double, Data>::const_iterator Iterator;

  explicit TimeMarkedList(double memory_cycle_sec = 0.0) {
    memory_cycle_sec_ = std::max(0.0, memory_cycle_sec);
  }

  double MemoryCycleSec() const { return memory_cycle_sec_; }

  bool Push(double timestamp_sec, const Data& data) {
    return Push(timestamp_sec, Data(data));
  }

  bool Push(double timestamp_sec, Data&& data) {
    auto latest = Latest();
    if (latest != std::map<double, Data>::end() &&
        latest->first >= timestamp_sec) {
      return false;
    }

    std::map<double, Data>::emplace(timestamp_sec, std::move(data));

    auto oldest = Oldest();
    if (oldest != std::map<double, Data>::end()) {
      auto oldest1 = oldest;
      oldest1++;
      if (oldest1 != std::map<double, Data>::end() &&
          timestamp_sec - oldest->first > memory_cycle_sec_ &&
          timestamp_sec - oldest1->first >= memory_cycle_sec_) {
        std::map<double, Data>::erase(oldest);
      }
    }

    return true;
  }

  bool Pop() {
    auto latest = Latest();
    if (latest == std::map<double, Data>::end()) {
      return false;
    }
    std::map<double, Data>::erase(latest);
    return true;
  }

  Iterator Latest() const {
    if (!std::map<double, Data>::empty()) {
      auto iter = std::map<double, Data>::end();
      iter--;
      return iter;
    } else {
      return std::map<double, Data>::end();
    }
  }

  Iterator Oldest() const { return std::map<double, Data>::begin(); }

  template <class T>
  bool Older(const TimeMarkedList<T>& other) const {
    auto it = other.Latest();
    if (it != other.end()) {
      return Older(it->first);
    }
    return false;
  }

  bool Older(double timestamp_sec) const {
    auto it = Latest();
    if (it != std::map<double, Data>::end()) {
      return it->first < timestamp_sec;
    }
    return false;
  }

  template <class T>
  bool Newer(const TimeMarkedList<T>& other) const {
    auto it = other.Latest();
    if (it != other.end()) {
      return Newer(it->first);
    }
    return false;
  }

  bool Newer(double timestamp_sec) const {
    auto it = Latest();
    if (it != std::map<double, Data>::end()) {
      return it->first > timestamp_sec;
    }
    return false;
  }

  std::pair<Iterator, Iterator> RangeOf(double timestamp_sec) const {
    auto upper = std::map<double, Data>::lower_bound(timestamp_sec);
    Iterator lower;
    if (upper != std::map<double, Data>::end()) {
      if (upper == std::map<double, Data>::begin()) {
        lower = std::map<double, Data>::end();
      } else {
        lower = upper;
        lower--;
      }
    } else {
      if (!std::map<double, Data>::empty()) {
        lower = std::map<double, Data>::end();
        lower--;
      } else {
        lower = std::map<double, Data>::end();
      }
    }
    return std::make_pair(lower, upper);
  }

  Iterator Nearest(double timestamp_sec) const {
    auto p = RangeOf(timestamp_sec);
    if (p.first == std::map<double, Data>::end() &&
        p.second == std::map<double, Data>::end()) {
      return std::map<double, Data>::end();
    }
    if (p.first == std::map<double, Data>::end()) {
      return p.second;
    } else if (p.second == std::map<double, Data>::end()) {
      return p.first;
    }
    if (timestamp_sec - p.first->first < p.second->first - timestamp_sec) {
      return p.first;
    } else {
      return p.second;
    }
  }

 private:
  double memory_cycle_sec_;
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_LMD_COMMON_TM_LIST_H_
