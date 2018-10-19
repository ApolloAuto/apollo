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
#include <cstring>
#include <mutex>
#include <string>

#include "cyber/base/atomic_rw_lock.h"

namespace apollo {
namespace cyber {
namespace transport {

using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;

class Block {
 public:
  Block();
  virtual ~Block();

  bool TryLockForWrite();
  bool TryLockForRead();
  void ReleaseWriteLock();
  void ReleaseReadLock();

  bool Write(uint8_t* dst, const std::string& msg, const std::string& msg_info);
  bool Read(const uint8_t* src, std::string* msg, std::string* msg_info);

 private:
  std::atomic<bool> is_writing_;
  std::atomic<uint32_t> reading_reference_counts_;
  base::AtomicRWLock read_write_mutex_;

  uint64_t msg_size_;
  uint64_t msg_info_size_;
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_SHM_BLOCK_H_
