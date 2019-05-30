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

#include "modules/bridge/common/bridge_buffer.h"

#include <memory.h>

namespace apollo {
namespace bridge {

#define BRIDGE_IMPL(type) template class BridgeBuffer<type>

template <typename T>
BridgeBuffer<T>::BridgeBuffer() {}

template <typename T>
BridgeBuffer<T>::BridgeBuffer(size_t size) {
  reset(size);
}

template <typename T>
BridgeBuffer<T>::~BridgeBuffer() {
  std::lock_guard<std::mutex> lg(mutex_);
  if (buf_) {
    delete[] buf_;
  }
  buf_ = nullptr;
  size_ = 0;
  capacity_ = 0;
}

template <typename T>
BridgeBuffer<T>::operator T *() {
  return buf_;
}

template <typename T>
void BridgeBuffer<T>::reset(size_t size) {
  std::lock_guard<std::mutex> lg(mutex_);
  if (capacity_ < size) {
    if (buf_) {
      delete[] buf_;
    }
    capacity_ = size;
    buf_ = new T[capacity_];
  }
  size_ = size;
  memset(buf_, 0, sizeof(T) * capacity_);
}

template <typename T>
void BridgeBuffer<T>::write(size_t index, const T *data, size_t size) {
  std::lock_guard<std::mutex> lg(mutex_);
  reset(size + index);
  T *p = buf_ + index;
  memcpy(p, data, size);
}

BRIDGE_IMPL(char);
BRIDGE_IMPL(int);
BRIDGE_IMPL(double);

}  // namespace bridge
}  // namespace apollo
