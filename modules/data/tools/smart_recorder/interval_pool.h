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

#include "absl/strings/str_cat.h"
#include "cyber/common/macros.h"

namespace apollo {
namespace data {

struct Interval {
  uint64_t begin_time;
  uint64_t end_time;
};

/**
 * @class IntervalPool
 * @brief The intervals collection class that organizes the intervals
 */
class IntervalPool {
 public:
  void AddInterval(const Interval& interval);
  void AddInterval(const uint64_t begin_time, const uint64_t end_time);
  void ReorgIntervals();
  bool MessageFallIntoRange(const uint64_t msg_time);
  void Reset();
  void PrintIntervals() const;
  Interval GetNextInterval() const;
  void SetIntervalEventLogFilePath(const std::string& path,
                                   const std::string& task_id) {
    interval_event_log_file_path_ = absl::StrCat(path, "_", task_id);
  }
  void LogIntervalEvent(const std::string& name, const std::string& description,
                        const uint64_t msg_time, const uint64_t backward_time,
                        const uint64_t forward_time) const;

 private:
  std::vector<Interval> pool_;
  std::vector<Interval>::iterator pool_iter_;
  std::set<uint64_t> accu_end_values_;
  std::string interval_event_log_file_path_;

  DECLARE_SINGLETON(IntervalPool)
};

}  // namespace data
}  // namespace apollo
