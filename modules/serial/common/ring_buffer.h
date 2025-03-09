// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#pragma once

#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace serial {

class RingBuffer {
 public:
  RingBuffer(size_t capacity = 256)
      : capacity_(capacity), buffer_(capacity), head_(0), tail_(0), count_(0) {}

  size_t Write(const uint8_t* data, size_t size) {
    CHECK_NOTNULL(data);
    size_t written_bytes = 0;
    for (size_t i = 0; i < size; ++i) {
      if (IsFull()) {
        AWARN << "RingBuffer is full, data will be overwritten.";
        head_ = (head_ + 1) % capacity_;
        count_--;
      }
      buffer_[tail_] = data[i];
      tail_ = (tail_ + 1) % capacity_;
      count_++;
      written_bytes++;
    }
    return written_bytes;
  }

  size_t Read(uint8_t* data, size_t size) {
    CHECK_NOTNULL(data);
    size_t read_bytes = 0;
    for (size_t i = 0; i < size; ++i) {
      if (IsEmpty()) {
        break;
      }
      data[i] = buffer_[head_];
      head_ = (head_ + 1) % capacity_;
      count_--;
      read_bytes++;
    }
    return read_bytes;
  }

  size_t Count() const { return count_; }

  size_t Capacity() const { return capacity_; }

  bool IsEmpty() const { return count_ == 0; }

  bool IsFull() const { return count_ == capacity_; }

 private:
  size_t capacity_;
  std::vector<uint8_t> buffer_;
  size_t head_;
  size_t tail_;
  size_t count_;
};

}  // namespace serial
}  // namespace apollo
