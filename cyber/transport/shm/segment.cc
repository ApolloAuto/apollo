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

#include "cyber/transport/shm/segment.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "cyber/common/util.h"
#include "cyber/transport/shm/shm_conf.h"

namespace apollo {
namespace cyber {
namespace transport {

Segment::Segment(uint64_t channel_id, const ReadWriteMode& mode)
    : init_(false),
      mode_(mode),
      conf_(),
      state_(nullptr),
      blocks_(nullptr),
      managed_shm_(nullptr),
      block_buf_lock_(),
      block_buf_addrs_() {
  id_ = static_cast<key_t>(channel_id);
}

Segment::~Segment() { Destroy(); }

bool Segment::AcquireBlockToWrite(std::size_t msg_size,
                                  WritableBlock* writable_block) {
  RETURN_VAL_IF_NULL(writable_block, false);
  if (!init_ && !Init()) {
    AERROR << "init failed, can't write now.";
    return false;
  }

  bool result = true;
  if (state_->need_remap()) {
    result = Remap();
  }

  if (msg_size > conf_.ceiling_msg_size()) {
    conf_.Update(msg_size);
    result = Recreate();
  }

  if (!result) {
    AERROR << "segment update failed.";
    return false;
  }

  uint32_t index = GetNextWritableBlockIndex();
  writable_block->index = index;
  writable_block->block = &blocks_[index];
  writable_block->buf = block_buf_addrs_[index];
  return true;
}

void Segment::ReleaseWrittenBlock(const WritableBlock& writable_block) {
  auto index = writable_block.index;
  if (index >= conf_.block_num()) {
    return;
  }
  blocks_[index].ReleaseWriteLock();
}

bool Segment::AcquireBlockToRead(ReadableBlock* readable_block) {
  RETURN_VAL_IF_NULL(readable_block, false);

  if (!init_ && !Init()) {
    AERROR << "init failed, can't read now.";
    return false;
  }
  auto index = readable_block->index;
  if (index >= conf_.block_num()) {
    AERROR << "invalid block_index[" << index << "].";
    return false;
  }

  bool result = true;
  if (state_->need_remap()) {
    result = Remap();
  }

  if (!result) {
    AERROR << "segment update failed.";
    return false;
  }

  if (!blocks_[index].TryLockForRead()) {
    return false;
  }
  readable_block->block = blocks_ + index;
  readable_block->buf = block_buf_addrs_[index];
  return true;
}

void Segment::ReleaseReadBlock(const ReadableBlock& readable_block) {
  auto index = readable_block.index;
  if (index >= conf_.block_num()) {
    return;
  }
  blocks_[index].ReleaseReadLock();
}

bool Segment::Init() {
  if (mode_ == READ_ONLY) {
    return OpenOnly();
  } else {
    return OpenOrCreate();
  }
}

bool Segment::OpenOrCreate() {
  if (init_) {
    return true;
  }

  // create managed_shm_
  int retry = 0;
  int shmid = 0;
  while (retry < 2) {
    shmid = shmget(id_, conf_.managed_shm_size(), 0644 | IPC_CREAT | IPC_EXCL);
    if (shmid != -1) {
      break;
    }

    if (EINVAL == errno) {
      AINFO << "need larger space, recreate.";
      Reset();
      Remove();
      ++retry;
    } else if (EEXIST == errno) {
      ADEBUG << "shm already exist, open only.";
      return OpenOnly();
    } else {
      break;
    }
  }

  if (shmid == -1) {
    AERROR << "create shm failed, error code: " << strerror(errno);
    return false;
  }

  // attach managed_shm_
  managed_shm_ = shmat(shmid, nullptr, 0);
  if (managed_shm_ == reinterpret_cast<void*>(-1)) {
    AERROR << "attach shm failed.";
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  // create field state_
  state_ = new (managed_shm_) State(conf_.ceiling_msg_size());
  if (state_ == nullptr) {
    AERROR << "create state failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  conf_.Update(state_->ceiling_msg_size());

  // create field blocks_
  blocks_ = new (static_cast<char*>(managed_shm_) + sizeof(State))
      Block[conf_.block_num()];
  if (blocks_ == nullptr) {
    AERROR << "create blocks failed.";
    state_->~State();
    state_ = nullptr;
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  // create block buf
  uint32_t i = 0;
  for (; i < conf_.block_num(); ++i) {
    uint8_t* addr =
        new (static_cast<char*>(managed_shm_) + sizeof(State) +
             conf_.block_num() * sizeof(Block) + i * conf_.block_buf_size())
            uint8_t[conf_.block_buf_size()];
    std::lock_guard<std::mutex> _g(block_buf_lock_);
    block_buf_addrs_[i] = addr;
  }

  if (i != conf_.block_num()) {
    AERROR << "create block buf failed.";
    state_->~State();
    state_ = nullptr;
    blocks_ = nullptr;
    {
      std::lock_guard<std::mutex> _g(block_buf_lock_);
      block_buf_addrs_.clear();
    }
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  state_->IncreaseReferenceCounts();
  init_ = true;
  ADEBUG << "open or create true.";
  return true;
}

bool Segment::OpenOnly() {
  if (init_) {
    return true;
  }

  // get managed_shm_
  int shmid = shmget(id_, 0, 0644);
  if (shmid == -1) {
    AERROR << "get shm failed.";
    return false;
  }

  // attach managed_shm_
  managed_shm_ = shmat(shmid, nullptr, 0);
  if (managed_shm_ == reinterpret_cast<void*>(-1)) {
    AERROR << "attach shm failed.";
    return false;
  }

  // get field state_
  state_ = reinterpret_cast<State*>(managed_shm_);
  if (state_ == nullptr) {
    AERROR << "get state failed.";
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    return false;
  }

  conf_.Update(state_->ceiling_msg_size());

  // get field blocks_
  blocks_ = reinterpret_cast<Block*>(static_cast<char*>(managed_shm_) +
                                     sizeof(State));
  if (blocks_ == nullptr) {
    AERROR << "get blocks failed.";
    state_ = nullptr;
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    return false;
  }

  // get block buf
  uint32_t i = 0;
  for (; i < conf_.block_num(); ++i) {
    uint8_t* addr = reinterpret_cast<uint8_t*>(
        static_cast<char*>(managed_shm_) + sizeof(State) +
        conf_.block_num() * sizeof(Block) + i * conf_.block_buf_size());

    if (addr == nullptr) {
      break;
    }

    std::lock_guard<std::mutex> _g(block_buf_lock_);
    block_buf_addrs_[i] = addr;
  }

  if (i != conf_.block_num()) {
    AERROR << "open only failed.";
    state_->~State();
    state_ = nullptr;
    blocks_ = nullptr;
    {
      std::lock_guard<std::mutex> _g(block_buf_lock_);
      block_buf_addrs_.clear();
    }
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
    shmctl(shmid, IPC_RMID, 0);
    return false;
  }

  state_->IncreaseReferenceCounts();
  init_ = true;
  ADEBUG << "open only true.";
  return true;
}

bool Segment::Remove() {
  int shmid = shmget(id_, 0, 0644);
  if (shmid == -1 || shmctl(shmid, IPC_RMID, 0) == -1) {
    AERROR << "remove shm failed, error code: " << strerror(errno);
    return false;
  }

  ADEBUG << "remove success.";
  return true;
}

bool Segment::Destroy() {
  if (!init_) {
    return true;
  }
  init_ = false;

  try {
    state_->DecreaseReferenceCounts();
    uint32_t reference_counts = state_->reference_counts();
    if (reference_counts == 0) {
      return Remove();
    }
  } catch (...) {
    AERROR << "exception.";
    return false;
  }
  ADEBUG << "destroy.";
  return true;
}

void Segment::Reset() {
  state_ = nullptr;
  blocks_ = nullptr;
  {
    std::lock_guard<std::mutex> _g(block_buf_lock_);
    block_buf_addrs_.clear();
  }

  if (managed_shm_ != nullptr) {
    shmdt(managed_shm_);
    managed_shm_ = nullptr;
  }
}

bool Segment::Remap() {
  init_ = false;
  ADEBUG << "before reset.";
  Reset();
  ADEBUG << "after reset.";
  return OpenOnly();
}

bool Segment::Recreate() {
  init_ = false;
  state_->set_need_remap(true);
  Reset();
  Remove();
  return OpenOrCreate();
}

uint32_t Segment::GetNextWritableBlockIndex() {
  uint32_t try_idx = state_->wrote_num();

  auto max_mod_num = conf_.block_num() - 1;
  while (1) {
    if (try_idx >= conf_.block_num()) {
      try_idx &= max_mod_num;
    }

    if (blocks_[try_idx].TryLockForWrite()) {
      state_->IncreaseWroteNum();
      return try_idx;
    }

    ++try_idx;
  }
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
