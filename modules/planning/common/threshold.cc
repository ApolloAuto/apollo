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

#include "modules/planning/common/threshold.h"

bool ThresholdItem::InRange(const uint32_t seq_num, const double value) {
  if (seq_num < last_seq_num) {
    return false;
  }
  if (seq_num - last_seq_num > max_time_interval) {
    return false;
  }
  last_seq_num = seq_num;
  if (value < min_val || value > max_val) {
    if (last_result == false) {
      ++repeat_time;
      last_result = false;
      if (repeat_time > max_repeat_time) {
        repeat_time = 0;
        return false;
      }
    }
    last_result = false;
    return false;
  }
  repeat_time = 0;
  last_result = true;
  return true;
}

void Threshold::AddItem(const std::string& id,
               const uint32_t seq_num,
               const double min_val,
               const double max_val,
               const uint32_t max_repeat_time,
               const uint32_t max_time_interval) {
  ThresholdItem& item = map_[id];
  item.repeat_time = max_repeat_time;
  item.last_seq_num = seq_num;
  item.last_result = false;
  item.min_val = min_val;
  item.max_val = max_val;
  item.max_repeat_time = max_repeat_time;
  item.max_time_interval = max_time_interval;
}

bool Threshold::IsInRange(const std::string& id,
                 const uint32_t seq_num,
                 const double value) {
  if (map_.find(id) == map_.end()) {
    return false;
  }
  return map_[id].InRange(seq_num, value);
}

