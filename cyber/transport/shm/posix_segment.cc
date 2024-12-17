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

#include "cyber/transport/shm/posix_segment.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "cyber/common/log.h"
#include "cyber/common/util.h"
#include "cyber/transport/shm/block.h"
#include "cyber/transport/shm/segment.h"
#include "cyber/transport/shm/shm_conf.h"

namespace apollo {
namespace cyber {
namespace transport {

PosixSegment::PosixSegment(uint64_t channel_id) : Segment(channel_id) {
  shm_name_ = std::to_string(channel_id);
}

PosixSegment::~PosixSegment() { Destroy(); }

bool PosixSegment::OpenOrCreate() {
  if (init_) {
    return true;
  }

  // create managed_shm_
  int fd = shm_open(shm_name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0644);
  if (fd < 0) {
    if (EEXIST == errno) {
      ADEBUG << "shm already exist, open only.";
      return OpenOnly();
    } else {
      AERROR << "create shm failed, error: " << strerror(errno);
      return false;
    }
  }

  if (ftruncate(fd, conf_.managed_shm_size()) < 0) {
    AERROR << "ftruncate failed: " << strerror(errno);
    close(fd);
    return false;
  }

  // attach managed_shm_
  managed_shm_ = mmap(nullptr, conf_.managed_shm_size(), PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
  if (managed_shm_ == MAP_FAILED) {
    AERROR << "attach shm failed:" << strerror(errno);
    close(fd);
    shm_unlink(shm_name_.c_str());
    return false;
  }

  close(fd);

  // create field state_
  state_ = new (managed_shm_) State(conf_.ceiling_msg_size());
  if (state_ == nullptr) {
    AERROR << "create state failed.";
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    shm_unlink(shm_name_.c_str());
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
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    shm_unlink(shm_name_.c_str());
    return false;
  }

  // create field arena_blocks_
  arena_blocks_ = new (static_cast<char*>(managed_shm_) + sizeof(State) + \
                       conf_.block_num() * sizeof(Block)) Block[
                        ShmConf::ARENA_BLOCK_NUM];
  if (arena_blocks_ == nullptr) {
    AERROR << "create blocks failed.";
    state_->~State();
    state_ = nullptr;
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    shm_unlink(shm_name_.c_str());
    return false;
  }

  // create block buf
  uint32_t i = 0;
  for (; i < conf_.block_num(); ++i) {
    uint8_t* addr =
        new (static_cast<char*>(managed_shm_) + sizeof(State) + \
             conf_.block_num() * sizeof(Block) + \
             ShmConf::ARENA_BLOCK_NUM * sizeof(Block) + \
             i * conf_.block_buf_size()) uint8_t[conf_.block_buf_size()];

    if (addr == nullptr) {
      break;
    }

    std::lock_guard<std::mutex> lg(block_buf_lock_);
    block_buf_addrs_[i] = addr;
  }

  // create arena block buf
  uint32_t ai = 0;
  for (; ai < ShmConf::ARENA_BLOCK_NUM; ++ai) {
    uint8_t* addr = \
        new(static_cast<char*>(managed_shm_) + sizeof(State) + \
             conf_.block_num() * sizeof(Block) + \
             ShmConf::ARENA_BLOCK_NUM * sizeof(Block) + \
             conf_.block_num() * conf_.block_buf_size() + \
             ai * ShmConf::ARENA_MESSAGE_SIZE) uint8_t[
              ShmConf::ARENA_MESSAGE_SIZE];
    if (addr == nullptr) {
      break;
    }

    std::lock_guard<std::mutex> _g(arena_block_buf_lock_);
    arena_block_buf_addrs_[ai] = addr;
  }

  if (ai != ShmConf::ARENA_BLOCK_NUM || i != conf_.block_num()) {
    AERROR << "create arena block buf failed.";
    state_->~State();
    state_ = nullptr;
    blocks_ = nullptr;
    arena_blocks_ = nullptr;
    {
      std::lock_guard<std::mutex> lg(block_buf_lock_);
      block_buf_addrs_.clear();
    }
    {
      std::lock_guard<std::mutex> lg(arena_block_buf_lock_);
      arena_block_buf_addrs_.clear();
    }
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    shm_unlink(shm_name_.c_str());
    return false;
  }

  state_->IncreaseReferenceCounts();
  init_ = true;
  return true;
}

bool PosixSegment::OpenOnly() {
  if (init_) {
    return true;
  }

  // get managed_shm_
  int fd = shm_open(shm_name_.c_str(), O_RDWR, 0644);
  if (fd == -1) {
    AERROR << "get shm failed: " << strerror(errno);
    return false;
  }

  struct stat file_attr;
  if (fstat(fd, &file_attr) < 0) {
    AERROR << "fstat failed: " << strerror(errno);
    close(fd);
    return false;
  }

  // attach managed_shm_
  managed_shm_ = mmap(nullptr, file_attr.st_size, PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
  if (managed_shm_ == MAP_FAILED) {
    AERROR << "attach shm failed: " << strerror(errno);
    close(fd);
    return false;
  }

  close(fd);
  // get field state_
  state_ = reinterpret_cast<State*>(managed_shm_);
  if (state_ == nullptr) {
    AERROR << "get state failed.";
    munmap(managed_shm_, file_attr.st_size);
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
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    return false;
  }

  // get field arena_blocks_
  arena_blocks_ = reinterpret_cast<Block*>(
    static_cast<char*>(managed_shm_) + sizeof(State) +
    sizeof(Block) * conf_.block_num());
  if (blocks_ == nullptr) {
    AERROR << "get arena blocks failed.";
    state_ = nullptr;
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    return false;
  }

  // get block buf
  uint32_t i = 0;
  for (; i < conf_.block_num(); ++i) {
    uint8_t* addr = reinterpret_cast<uint8_t*>(
        static_cast<char*>(managed_shm_) + sizeof(State) +
        conf_.block_num() * sizeof(Block) +
        ShmConf::ARENA_BLOCK_NUM * sizeof(Block) +
        i * conf_.block_buf_size());

    if (addr == nullptr) {
      break;
    }
    std::lock_guard<std::mutex> lg(block_buf_lock_);
    block_buf_addrs_[i] = addr;
  }

  // get arena block buf
  uint32_t ai = 0;
  for (; i < ShmConf::ARENA_BLOCK_NUM; ++ai) {
    uint8_t* addr = reinterpret_cast<uint8_t*>(
        static_cast<char*>(managed_shm_) + sizeof(State) + \
        conf_.block_num() * sizeof(Block) + ShmConf::ARENA_BLOCK_NUM * \
        sizeof(Block) + conf_.block_num() * conf_.block_buf_size() + \
        ai * ShmConf::ARENA_MESSAGE_SIZE);

    if (addr == nullptr) {
      break;
    }
    std::lock_guard<std::mutex> _g(arena_block_buf_lock_);
    arena_block_buf_addrs_[ai] = addr;
  }

  if (i != conf_.block_num() || ai != ShmConf::ARENA_BLOCK_NUM) {
    AERROR << "open only failed.";
    state_->~State();
    state_ = nullptr;
    blocks_ = nullptr;
    arena_blocks_ = nullptr;
    {
      std::lock_guard<std::mutex> lg(block_buf_lock_);
      block_buf_addrs_.clear();
    }
    {
      std::lock_guard<std::mutex> lg(arena_block_buf_lock_);
      arena_block_buf_addrs_.clear();
    }
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    shm_unlink(shm_name_.c_str());
    return false;
  }

  state_->IncreaseReferenceCounts();
  init_ = true;
  ADEBUG << "open only true.";
  return true;
}

bool PosixSegment::Remove() {
  if (shm_unlink(shm_name_.c_str()) < 0) {
    AERROR << "shm_unlink failed: " << strerror(errno);
    return false;
  }
  return true;
}

void PosixSegment::Reset() {
  state_ = nullptr;
  blocks_ = nullptr;
  arena_blocks_ = nullptr;
  {
    std::lock_guard<std::mutex> lg(block_buf_lock_);
    block_buf_addrs_.clear();
  }
  {
    std::lock_guard<std::mutex> lg(arena_block_buf_lock_);
    arena_block_buf_addrs_.clear();
  }
  if (managed_shm_ != nullptr) {
    munmap(managed_shm_, conf_.managed_shm_size());
    managed_shm_ = nullptr;
    return;
  }
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
