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

#include "modules/common/filters/mean_filter.h"

#include <limits>

#include "cyber/common/log.h"

namespace apollo {
namespace common {

using MF = MeanFilter;
using uint8 = std::uint_fast8_t;
using TimedValue = std::pair<uint8, double>;

const uint8 kMaxWindowSize = std::numeric_limits<uint8>::max() / 2;

MF::MeanFilter(const uint8 window_size) : window_size_(window_size) {
  CHECK_GT(window_size_, 0);
  CHECK_LE(window_size_, kMaxWindowSize);
  initialized_ = true;
}

double MF::GetMin() const {
  if (min_candidates_.empty()) {
    return std::numeric_limits<double>::infinity();
  } else {
    return min_candidates_.front().second;
  }
}

double MF::GetMax() const {
  if (max_candidates_.empty()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return max_candidates_.front().second;
  }
}

double MF::Update(const double measurement) {
  CHECK(initialized_);
  CHECK_LE(values_.size(), window_size_);
  CHECK_LE(min_candidates_.size(), window_size_);
  CHECK_LE(max_candidates_.size(), window_size_);
  ++time_;
  time_ %= static_cast<std::uint_fast8_t>(2 * window_size_);
  if (values_.size() == window_size_) {
    RemoveEarliest();
  }
  Insert(measurement);
  if (values_.size() > 2) {
    return (sum_ - GetMin() - GetMax()) /
           static_cast<double>(values_.size() - 2);
  } else {
    return sum_ / static_cast<double>(values_.size());
  }
}

bool MF::ShouldPopOldestCandidate(const uint8 old_time) const {
  if (old_time < window_size_) {
    CHECK_LE(time_, old_time + window_size_);
    return old_time + window_size_ == time_;
  } else if (time_ < window_size_) {
    CHECK_GE(old_time, time_ + window_size_);
    return old_time == time_ + window_size_;
  } else {
    return false;
  }
}

void MF::RemoveEarliest() {
  CHECK_EQ(values_.size(), window_size_);
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (ShouldPopOldestCandidate(min_candidates_.front().first)) {
    min_candidates_.pop_front();
  }
  if (ShouldPopOldestCandidate(max_candidates_.front().first)) {
    max_candidates_.pop_front();
  }
}

void MF::Insert(const double value) {
  values_.push_back(value);
  sum_ += value;
  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));
  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}

}  // namespace common
}  // namespace apollo
