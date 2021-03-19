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
#include <fstream>
#include <iomanip>

#include "cyber/common/log.h"

namespace apollo {
namespace data {

IntervalPool::IntervalPool() {}

void IntervalPool::AddInterval(const Interval& interval) {
  if (pool_.empty() || interval.begin_time > pool_iter_->end_time) {
    pool_.push_back(interval);
    pool_iter_ = std::prev(pool_.end());
    return;
  }
  pool_iter_->begin_time =
      std::min(interval.begin_time, pool_iter_->begin_time);
  pool_iter_->end_time = std::max(interval.end_time, pool_iter_->end_time);
}

void IntervalPool::AddInterval(const uint64_t begin_time,
                               const uint64_t end_time) {
  struct Interval interval;
  interval.begin_time = begin_time;
  interval.end_time = end_time;
  AddInterval(interval);
}

void IntervalPool::ReorgIntervals() {
  // Sort the intervals by begin_time ascending
  std::sort(pool_.begin(), pool_.end(),
            [](const Interval& x, const Interval& y) {
              return x.begin_time < y.begin_time;
            });
  pool_iter_ = pool_.begin();
  accu_end_values_.clear();
}

bool IntervalPool::MessageFallIntoRange(const uint64_t msg_time) {
  // For each message comes for checking, the logic is:
  // 1. Add end_time of any intervals whose begin_time is smaller
  //    than message time to the helper set
  // 2. Now if the helper set is not empty, means some range is still
  //    in progress, returns true
  // 3. After this, remove end_time equals to message time from the set,
  //    which means these ranges are done being used
  // This way range groups iterate along with messages, time complexity O(N)
  while (pool_iter_ != pool_.end() && msg_time >= pool_iter_->begin_time) {
    accu_end_values_.insert(pool_iter_->end_time);
    ++pool_iter_;
  }
  bool found = !accu_end_values_.empty();
  accu_end_values_.erase(msg_time);
  return found;
}

void IntervalPool::Reset() {
  pool_.clear();
  pool_iter_ = pool_.begin();
  accu_end_values_.clear();
}

void IntervalPool::PrintIntervals() const {
  auto idx = 0;
  for (const auto& interval : pool_) {
    AINFO << "Interval " << ++idx << ": " << interval.begin_time << " - "
          << interval.end_time;
  }
}

void IntervalPool::LogIntervalEvent(const std::string& name,
                                    const std::string& description,
                                    const uint64_t msg_time,
                                    const uint64_t backward_time,
                                    const uint64_t forward_time) const {
  std::ofstream logfile(interval_event_log_file_path_,
                        std::ios::out | std::ios::app);
  if (!logfile) {
    AERROR << "Failed to write " << interval_event_log_file_path_;
    return;
  }
  logfile << std::fixed << std::setprecision(9);
  logfile << "name=" << name << ", description=\"" << description << "\""
          << ", msg_time=" << msg_time << ", interval_range=["
          << msg_time - backward_time << ":" << msg_time + forward_time << "]"
          << std::endl;
}

Interval IntervalPool::GetNextInterval() const {
  if (pool_.empty()) {
    struct Interval interval;
    interval.begin_time = 0;
    interval.end_time = 0;
    return interval;
  }
  return *pool_iter_;
}

}  // namespace data
}  // namespace apollo
