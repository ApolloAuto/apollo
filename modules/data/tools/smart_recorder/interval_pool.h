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

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/macros.h"

namespace apollo {
namespace data {

struct Interval {
  std::set<std::string> channels;
  uint64_t begin_time;
  uint64_t end_time;
};

/**
 * @class IntervalPool
 * @brief The intervals collection class that organizes the intervals
 */
class IntervalPool {
 public:
  void AddInterval(const Interval interval) { pool_.push_back(interval); }
  void AddInterval(
      const uint64_t begin_time, const uint64_t end_time,
      const std::unordered_map<std::string, std::string>& channels);
  void ReorgIntervals();
  bool MessageFallIntoRange(const std::string& msg_channel,
                            const uint64_t msg_time);
  void Reset();
  void PrintIntervals() const;

 private:
  void AddNewIntervalChannels(const std::set<std::string>& channels);
  void EraseOldIntervalChannels(const std::set<std::string>& channels);

  std::vector<Interval> pool_;
  std::vector<Interval>::const_iterator pool_iter_;
  std::multimap<uint64_t, Interval> accu_intervals_;
  std::unordered_map<std::string, int> accu_channels_;

  DECLARE_SINGLETON(IntervalPool)
};

}  // namespace data
}  // namespace apollo

