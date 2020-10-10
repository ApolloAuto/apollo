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

#ifndef CYBER_TRANSPORT_SHM_BLOCK_H_
#define CYBER_TRANSPORT_SHM_BLOCK_H_

#include <atomic>
#include <cstdint>

namespace apollo {
namespace cyber {
namespace transport {

class Block {
  friend class Segment;

 public:
  Block();
  virtual ~Block();

  uint64_t msg_size() const { return msg_size_; }
  void set_msg_size(uint64_t msg_size) { msg_size_ = msg_size; }

  uint64_t msg_info_size() const { return msg_info_size_; }
  void set_msg_info_size(uint64_t msg_info_size) {
    msg_info_size_ = msg_info_size;
  }

  static const int32_t kRWLockFree;
  static const int32_t kWriteExclusive;
  static const int32_t kMaxTryLockTimes;

 private:
  bool TryLockForWrite();
  bool TryLockForRead();
  void ReleaseWriteLock();
  void ReleaseReadLock();

  volatile std::atomic<int32_t> lock_num_ = {0};

  uint64_t msg_size_;
  uint64_t msg_info_size_;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_SHM_BLOCK_H_
