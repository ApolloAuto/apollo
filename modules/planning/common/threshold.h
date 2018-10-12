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
#pragma once

#include <string>
#include <unordered_map>

struct ThresholdItem {
  ThresholdItem() = default;
  uint32_t repeat_time = 0;
  uint32_t max_repeat_time = 0;
  uint32_t max_time_interval = 1;
  uint32_t last_seq_num = 0;
  bool last_result = false;
  double min_val = 0.0;
  double max_val = 0.0;
  bool InRange(const uint32_t seq_num, const double value);
};

class Threshold {
 public:
  Threshold() = default;
  ~Threshold() = default;
  void AddItem(const std::string& id,
               const uint32_t seq_num,
               const double min_val,
               const double max_val,
               const uint32_t max_repeat_time,
               const uint32_t max_time_interval);
  bool IsInRange(const std::string& id,
                 const uint32_t seq_num,
                 const double value);
 private:
  std::unordered_map<std::string, ThresholdItem> map_;
};
