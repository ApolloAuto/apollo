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

#ifndef CYBER_TRANSPORT_SHM_SEGMENT_H_
#define CYBER_TRANSPORT_SHM_SEGMENT_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "cyber/transport/shm/block.h"
#include "cyber/transport/shm/shm_conf.h"
#include "cyber/transport/shm/state.h"

namespace apollo {
namespace cyber {
namespace transport {

class Segment;
using SegmentPtr = std::shared_ptr<Segment>;

struct WritableBlock {
  uint32_t index = 0;
  Block* block = nullptr;
  uint8_t* buf = nullptr;
};
using ReadableBlock = WritableBlock;

class Segment {
 public:
  explicit Segment(uint64_t channel_id);
  virtual ~Segment() {}

  bool AcquireBlockToWrite(std::size_t msg_size, WritableBlock* writable_block);
  void ReleaseWrittenBlock(const WritableBlock& writable_block);

  bool AcquireArenaBlockToWrite(
    std::size_t msg_size, WritableBlock* writable_block);
  void ReleaseArenaWrittenBlock(const WritableBlock& writable_block);

  bool AcquireBlockToRead(ReadableBlock* readable_block);
  void ReleaseReadBlock(const ReadableBlock& readable_block);

  bool AcquireArenaBlockToRead(ReadableBlock* readable_block);
  void ReleaseArenaReadBlock(const ReadableBlock& readable_block);

  bool InitOnly(uint64_t message_size);
  void* GetManagedShm();
  bool LockBlockForWriteByIndex(uint64_t block_index);
  bool ReleaseBlockForWriteByIndex(uint64_t block_index);
  bool LockBlockForReadByIndex(uint64_t block_index);
  bool ReleaseBlockForReadByIndex(uint64_t block_index);

  bool LockArenaBlockForWriteByIndex(uint64_t block_index);
  bool ReleaseArenaBlockForWriteByIndex(uint64_t block_index);
  bool LockArenaBlockForReadByIndex(uint64_t block_index);
  bool ReleaseArenaBlockForReadByIndex(uint64_t block_index);

 protected:
  virtual bool Destroy();
  virtual void Reset() = 0;
  virtual bool Remove() = 0;
  virtual bool OpenOnly() = 0;
  virtual bool OpenOrCreate() = 0;

  bool init_;
  ShmConf conf_;
  uint64_t channel_id_;

  State* state_;
  Block* blocks_;
  Block* arena_blocks_;
  void* managed_shm_;
  std::mutex block_buf_lock_;
  std::mutex arena_block_buf_lock_;
  std::unordered_map<uint32_t, uint8_t*> block_buf_addrs_;
  std::unordered_map<uint32_t, uint8_t*> arena_block_buf_addrs_;

 private:
  bool Remap();
  bool Recreate(const uint64_t& msg_size);
  uint32_t GetNextWritableBlockIndex();
  uint32_t GetNextArenaWritableBlockIndex();
};

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_SHM_SEGMENT_H_
