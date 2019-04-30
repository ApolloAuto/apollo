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

#include "modules/data/tools/smart_recorder/interval_pool.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace apollo {
namespace data {

IntervalPool::IntervalPool() {}

void IntervalPool::AddInterval(
    const uint64_t begin_time, const uint64_t end_time,
    const std::unordered_map<std::string, std::string>& channels) {
  struct Interval interval;
  interval.begin_time = begin_time;
  interval.end_time = end_time;
  for (const auto& iter : channels) {
    interval.channels.insert(iter.first);
  }
  AddInterval(interval);
}

void IntervalPool::ReorgIntervals() {
  // Sort the intervals by begin_time ascending
  std::sort(pool_.begin(), pool_.end(),
            [](const Interval& x, const Interval& y) {
              return x.begin_time < y.begin_time;
            });
  pool_iter_ = pool_.begin();
  accu_intervals_.clear();
  accu_channels_.clear();
}

bool IntervalPool::MessageFallIntoRange(const std::string& msg_channel,
                                        const uint64_t msg_time) {
  // For each message comes for checking, the logic is:
  // 1. Add all ranges with begin_time smaller than message time to the helper
  //    accumulated intervals map
  // 2. Meanwhile add corresponding channels to helper accumulated channels map
  // 3. Now if the message's channel exists in current accumulated channels map,
  //    means it falls into some range, returns true
  // 4. After this, remove all ranges with end_time equals to message time,
  //    which means these ranges are done being used
  // 5. Meanwhile remove corresponding channels from accumulated channels map
  // This way range groups iterate along with messages, time complexity O(N)
  while (pool_iter_ != pool_.end() && msg_time >= pool_iter_->begin_time) {
    accu_intervals_.insert({pool_iter_->end_time, *pool_iter_});
    AddNewIntervalChannels(pool_iter_->channels);
    ++pool_iter_;
  }
  auto channel_iter = accu_channels_.find(msg_channel);
  const bool is_found =
      channel_iter != accu_channels_.end() && channel_iter->second > 0;
  auto interval_iter = accu_intervals_.equal_range(msg_time);
  for (auto it = interval_iter.first; it != interval_iter.second; ++it) {
    EraseOldIntervalChannels(it->second.channels);
    accu_intervals_.erase(it);
  }
  return is_found;
}

void IntervalPool::Reset() {
  pool_.clear();
  pool_iter_ = pool_.begin();
  accu_intervals_.clear();
  accu_channels_.clear();
}

void IntervalPool::PrintIntervals() const {
  auto idx = 0;
  for (const auto& interval : pool_) {
    AINFO << ++idx << ": " << interval.begin_time << " - " << interval.end_time;
    for (const std::string& channel : interval.channels) {
      AINFO << channel;
    }
  }
}

void IntervalPool::AddNewIntervalChannels(
    const std::set<std::string>& channels) {
  for (const std::string& add_channel : channels) {
    auto channel_iter = accu_channels_.find(add_channel);
    if (channel_iter == accu_channels_.end()) {
      accu_channels_.insert({add_channel, 1});
    } else {
      channel_iter->second += 1;
    }
  }
}

void IntervalPool::EraseOldIntervalChannels(
    const std::set<std::string>& channels) {
  for (const std::string& del_channel : channels) {
    auto channel_iter = accu_channels_.find(del_channel);
    if (channel_iter != accu_channels_.end()) {
      channel_iter->second -= 1;
    }
  }
}

}  // namespace data
}  // namespace apollo
