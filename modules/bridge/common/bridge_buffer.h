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

#include <mutex>

namespace apollo {
namespace bridge {

template <typename T>
class BridgeBuffer {
 public:
  BridgeBuffer();
  explicit BridgeBuffer(size_t size);
  virtual ~BridgeBuffer();

  operator T *();
  void reset(size_t size);
  size_t size() const { return size_; }
  size_t capacity() const { return capacity_; }
  void write(size_t index, const T *data, size_t size);

 private:
  T *buf_ = nullptr;
  size_t size_ = 0;
  size_t capacity_ = 0;
  std::mutex mutex_;

  BridgeBuffer(const BridgeBuffer &) = delete;
  BridgeBuffer &operator=(const BridgeBuffer &) = delete;
};

}  // namespace bridge
}  // namespace apollo
