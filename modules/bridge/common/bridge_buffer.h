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

// Put this in the declarations for a class to be uncopyable.
#define DISABLE_COPY(TypeName) TypeName(const TypeName &) = delete

// Put this in the declarations for a class to be unassignable.
#define DISABLE_ASSIGN(TypeName) void operator=(const TypeName &) = delete

// A macro to disallow the copy constructor and operator= functions.
// This should be used in the private: declarations for a class.
#define DISABLE_COPY_AND_ASSIGN(TypeName) \
  DISABLE_COPY(TypeName);                 \
  DISABLE_ASSIGN(TypeName)

template<typename T>
class BridgeBuffer {
 public:
  BridgeBuffer();
  explicit BridgeBuffer(unsigned int size);
  virtual ~BridgeBuffer();

  operator T* ();
  void reset(unsigned int size);
  unsigned int size() const { return size_;}
  unsigned int capacity() const { return capacity_;}

 private:
  T *buf_ = nullptr;
  unsigned int size_ = 0;
  unsigned int capacity_ = 0;
  std::mutex mutex_;
  DISABLE_COPY_AND_ASSIGN(BridgeBuffer);
};

}  // namespace bridge
}  // namespace apollo
