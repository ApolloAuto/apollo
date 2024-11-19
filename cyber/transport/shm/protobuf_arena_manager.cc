/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "cyber/transport/shm/protobuf_arena_manager.h"

#include <sys/ipc.h>
#include <sys/shm.h>

#include <cstring>
#include <string>

#include <google/protobuf/arena.h>
#include <google/protobuf/message.h>

#include "cyber/transport/shm/segment_factory.h"

namespace apollo {
namespace cyber {
namespace transport {

const int32_t ArenaSegmentBlock::kRWLockFree = 0;
const int32_t ArenaSegmentBlock::kWriteExclusive = -1;
const int32_t ArenaSegmentBlock::kMaxTryLockTimes = 5;

ArenaSegment::ArenaSegment()
    : channel_id_(0), key_id_(0), base_address_(nullptr) {}

ArenaSegment::ArenaSegment(uint64_t channel_id)
    : channel_id_(channel_id),
      key_id_(std::hash<std::string>{}("/apollo/__arena__/" +
                                       std::to_string(channel_id))) {}

ArenaSegment::ArenaSegment(uint64_t channel_id, void* base_address)
    : channel_id_(channel_id),
      key_id_(std::hash<std::string>{}("/apollo/__arena__/" +
                                       std::to_string(channel_id))),
      base_address_(base_address) {}

ArenaSegment::ArenaSegment(uint64_t channel_id, uint64_t message_size,
                           uint64_t block_num, void* base_address)
    : channel_id_(channel_id),
      key_id_(std::hash<std::string>{}("/apollo/__arena__/" +
                                       std::to_string(channel_id))),
      base_address_(base_address) {
  Init(message_size, block_num);
}

ArenaSegment::~ArenaSegment() {}

bool ArenaSegment::Init(uint64_t message_size, uint64_t block_num) {
  uint64_t key_id = std::hash<std::string>{}("/apollo/__arena__/" +
                                             std::to_string(channel_id_));
  // fprintf(stderr, "channel_id: %lx, key_id: %lx\n", channel_id_, key_id);

  for (uint32_t retry = 0; retry < 2 && !OpenOrCreate(message_size, block_num);
       ++retry) {
  }

  return true;
}

bool ArenaSegment::OpenOrCreate(uint64_t message_size, uint64_t block_num) {
  auto arena_conf =
      cyber::common::GlobalData::Instance()->GetChannelArenaConf(channel_id_);
  auto shared_buffer_size = arena_conf.shared_buffer_size();
  auto size = sizeof(ArenaSegmentState) +
              sizeof(ArenaSegmentBlockDescriptor) * block_num +
              message_size * block_num + shared_buffer_size;
  auto shmid =
      shmget(static_cast<key_t>(key_id_), size, 0644 | IPC_CREAT | IPC_EXCL);
  if (shmid == -1) {
    if (errno == EINVAL) {
      // TODO(all): need larger space, recreate
    } else if (errno == EEXIST) {
      // TODO(all): shm already exist, open only
      return Open(message_size, block_num);
    } else {
      //  create or open shm failed
      return false;
    }
  }
  message_capacity_ = message_size;
  shm_address_ = shmat(shmid, base_address_, 0);
  if (shm_address_ == reinterpret_cast<void*>(-1)) {
    // shmat failed
    return false;
  }

  arenas_.resize(block_num, nullptr);
  if (shared_buffer_size == 0) {
    shared_buffer_arena_ = nullptr;
  } else {
    google::protobuf::ArenaOptions options;
    options.start_block_size = shared_buffer_size;
    options.max_block_size = shared_buffer_size;
    options.initial_block = reinterpret_cast<char*>(
        reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState) +
        block_num * sizeof(ArenaSegmentBlock) + block_num * message_size);
    options.initial_block_size = shared_buffer_size;
    shared_buffer_arena_ = std::make_shared<google::protobuf::Arena>(options);
  }
  for (size_t i = 0; i < block_num; i++) {
    arena_block_address_.push_back(
        reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState) +
        block_num * sizeof(ArenaSegmentBlock) + i * message_size);
  }

  state_ = reinterpret_cast<ArenaSegmentState*>(shm_address_);
  state_->struct_.ref_count_.store(1);
  state_->struct_.auto_extended_.store(false);
  state_->struct_.message_size_.store(message_size);
  state_->struct_.block_num_.store(block_num);
  state_->struct_.message_seq_.store(0);
  blocks_ = reinterpret_cast<ArenaSegmentBlock*>(
      reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState));
  for (uint64_t i = 0; i < block_num; ++i) {
    blocks_[i].size_ = message_size;
    // blocks_[i].writing_ref_count_.store(0);
    // blocks_[i].reading_ref_count_.store(0);
    blocks_[i].lock_num_.store(0);
  }
  return true;
}

bool ArenaSegment::Open(uint64_t message_size, uint64_t block_num) {
  auto arena_conf =
      cyber::common::GlobalData::Instance()->GetChannelArenaConf(channel_id_);
  auto shared_buffer_size = arena_conf.shared_buffer_size();
  auto shmid = shmget(static_cast<key_t>(key_id_), 0, 0644);
  if (shmid == -1) {
    // shm not exist
    return false;
  }
  shm_address_ = shmat(shmid, base_address_, 0);
  if (shm_address_ == reinterpret_cast<void*>(-1)) {
    // shmat failed
    return false;
  }
  message_capacity_ = message_size;
  state_ = reinterpret_cast<ArenaSegmentState*>(shm_address_);
  state_->struct_.ref_count_.fetch_add(1);
  blocks_ = reinterpret_cast<ArenaSegmentBlock*>(
      reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState));

  arenas_.resize(block_num, nullptr);
  if (shared_buffer_size == 0) {
    shared_buffer_arena_ = nullptr;
  } else {
    google::protobuf::ArenaOptions options;
    options.start_block_size = shared_buffer_size;
    options.max_block_size = shared_buffer_size;
    options.initial_block = reinterpret_cast<char*>(
        reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState) +
        block_num * sizeof(ArenaSegmentBlock) + block_num * message_size);
    options.initial_block_size = shared_buffer_size;
    shared_buffer_arena_ = std::make_shared<google::protobuf::Arena>(options);
  }
  for (size_t i = 0; i < block_num; i++) {
    arena_block_address_.push_back(
        reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState) +
        block_num * sizeof(ArenaSegmentBlock) + i * message_size);
  }
  return true;
}

void* ArenaSegment::GetShmAddress() { return shm_address_; }

uint64_t ArenaSegment::GetNextWritableBlockIndex() {
  const auto block_num = state_->struct_.block_num_.load();
  while (1) {
    uint64_t next_idx = state_->struct_.message_seq_.fetch_add(1) % block_num;
    if (AddBlockWriteLock(next_idx)) {
      return next_idx;
    }
  }
  return 0;
}

bool ArenaSegment::AddBlockWriteLock(uint64_t block_index) {
  // base::WriteLockGuard<base::PthreadRWLock> lock(
  //     blocks_[block_index].read_write_mutex_);
  // if (blocks_[block_index].writing_ref_count_.load() > 0) {
  //   return false;
  // }
  // if (blocks_[block_index].reading_ref_count_.load() > 0) {
  //   return false;
  // }
  // blocks_[block_index].writing_ref_count_.fetch_add(1);
  auto& block = blocks_[block_index];
  int32_t rw_lock_free = ArenaSegmentBlock::kRWLockFree;
  if (!block.lock_num_.compare_exchange_weak(
          rw_lock_free, ArenaSegmentBlock::kWriteExclusive,
          std::memory_order_acq_rel, std::memory_order_relaxed)) {
    ADEBUG << "lock num: " << block.lock_num_.load();
    return false;
  }
  return true;
}

void ArenaSegment::RemoveBlockWriteLock(uint64_t block_index) {
  blocks_[block_index].lock_num_.fetch_add(1);
}

bool ArenaSegment::AddBlockReadLock(uint64_t block_index) {
  // when multiple readers are reading an arena channel
  // at the same time, using write locks can cause one
  // or more readers to hang at the lock, whereas
  // read locks do not have this problem
  // base::ReadLockGuard<base::PthreadRWLock> lock(
  //     blocks_[block_index].read_write_mutex_);
  // if (blocks_[block_index].writing_ref_count_.load() > 0) {
  //   return false;
  // }
  // blocks_[block_index].reading_ref_count_.fetch_add(1);
  auto& block = blocks_[block_index];
  int32_t lock_num = block.lock_num_.load();
  if (lock_num < ArenaSegmentBlock::kRWLockFree) {
    AINFO << "block is being written.";
    return false;
  }

  int32_t try_times = 0;
  while (!block.lock_num_.compare_exchange_weak(lock_num, lock_num + 1,
                                                std::memory_order_acq_rel,
                                                std::memory_order_relaxed)) {
    ++try_times;
    if (try_times == ArenaSegmentBlock::kMaxTryLockTimes) {
      AINFO << "fail to add read lock num, curr num: " << lock_num;
      return false;
    }

    lock_num = block.lock_num_.load();
    if (lock_num < ArenaSegmentBlock::kRWLockFree) {
      AINFO << "block is being written.";
      return false;
    }
  }
  return true;
}

void ArenaSegment::RemoveBlockReadLock(uint64_t block_index) {
  blocks_[block_index].lock_num_.fetch_sub(1);
}

bool ArenaSegment::AcquireBlockToWrite(uint64_t size,
                                       ArenaSegmentBlockInfo* block_info) {
  if (!block_info) {
    return false;
  }
  if (!state_ || !blocks_) {
    return false;
  }

  // TODO(all): support dynamic block size

  uint64_t block_num = state_->struct_.block_num_.load();
  uint64_t block_size = state_->struct_.message_size_.load();
  uint64_t block_index = GetNextWritableBlockIndex();
  block_info->block_index_ = block_index;
  block_info->block_ = &blocks_[block_index];
  block_info->block_buffer_address_ = reinterpret_cast<void*>(
      reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState) +
      block_num * sizeof(ArenaSegmentBlock) + block_index * block_size);
  return true;
}

void ArenaSegment::ReleaseWrittenBlock(
    const ArenaSegmentBlockInfo& block_info) {
  if (!state_ || !blocks_) {
    return;
  }
  if (block_info.block_index_ >= state_->struct_.block_num_.load()) {
    return;
  }
  RemoveBlockWriteLock(block_info.block_index_);
}

bool ArenaSegment::AcquireBlockToRead(ArenaSegmentBlockInfo* block_info) {
  if (!block_info) {
    return false;
  }
  if (!state_ || !blocks_) {
    return false;
  }

  if (block_info->block_index_ >= state_->struct_.block_num_.load()) {
    return false;
  }

  // TODO(all): support dynamic block size

  if (!AddBlockReadLock(block_info->block_index_)) {
    return false;
  }
  uint64_t block_num = state_->struct_.block_num_.load();
  uint64_t block_size = state_->struct_.message_size_.load();

  block_info->block_ = &blocks_[block_info->block_index_];
  block_info->block_buffer_address_ = reinterpret_cast<void*>(
      reinterpret_cast<uint64_t>(shm_address_) + sizeof(ArenaSegmentState) +
      block_num * sizeof(ArenaSegmentBlock) +
      block_info->block_index_ * block_size);
  return true;
}

void ArenaSegment::ReleaseReadBlock(const ArenaSegmentBlockInfo& block_info) {
  if (!state_ || !blocks_) {
    return;
  }
  if (block_info.block_index_ >= state_->struct_.block_num_.load()) {
    return;
  }
  RemoveBlockReadLock(block_info.block_index_);
}

ProtobufArenaManager::ProtobufArenaManager() {
  address_allocator_ = std::make_shared<ArenaAddressAllocator>();
}

ProtobufArenaManager::~ProtobufArenaManager() { Destroy(); }

uint64_t ProtobufArenaManager::GetBaseAddress(
    const message::ArenaMessageWrapper* wrapper) {
  return 0;
}

std::shared_ptr<ArenaSegment> ProtobufArenaManager::GetSegment(
    uint64_t channel_id) {
  std::lock_guard<std::mutex> lock(segments_mutex_);
  if (segments_.find(channel_id) == segments_.end()) {
    return nullptr;
  }
  return segments_[channel_id];
}

void* ProtobufArenaManager::SetMessage(message::ArenaMessageWrapper* wrapper,
                                       const void* message) {
  auto input_msg = reinterpret_cast<const google::protobuf::Message*>(message);
  auto channel_id = GetMessageChannelId(wrapper);
  auto segment = GetSegment(channel_id);
  auto arena_ptr = input_msg->GetArena();
  google::protobuf::ArenaOptions options;

  if (!segment) {
    return nullptr;
  }

  void* msg_output = nullptr;
  if (arena_ptr == nullptr) {
    auto arena_conf =
        cyber::common::GlobalData::Instance()->GetChannelArenaConf(channel_id);
    google::protobuf::ArenaOptions options;
    options.start_block_size = arena_conf.max_msg_size();
    options.max_block_size = arena_conf.max_msg_size();

    if (!segment) {
      return nullptr;
    }

    ResetMessageRelatedBlocks(wrapper);

    ArenaSegmentBlockInfo wb;
    // TODO(all): AcquireBlockToWrite for dynamic adjust block
    // auto size = input_msg->ByteSizeLong();
    uint64_t size = 0;
    if (!segment->AcquireBlockToWrite(size, &wb)) {
      return nullptr;
    }
    this->AddMessageRelatedBlock(wrapper, wb.block_index_);
    options.initial_block =
        reinterpret_cast<char*>(segment->arena_block_address_[wb.block_index_]);
    options.initial_block_size = segment->message_capacity_;
    if (segment->arenas_[wb.block_index_] != nullptr) {
      segment->arenas_[wb.block_index_] = nullptr;
    }
    segment->arenas_[wb.block_index_] =
        std::make_shared<google::protobuf::Arena>(options);
    auto msg = input_msg->New(segment->arenas_[wb.block_index_].get());
    msg->CopyFrom(*input_msg);
    SetMessageAddressOffset(
        wrapper, reinterpret_cast<uint64_t>(msg) -
                     reinterpret_cast<uint64_t>(segment->GetShmAddress()));
    msg_output = reinterpret_cast<void*>(msg);
    segment->ReleaseWrittenBlock(wb);
  } else {
    ArenaSegmentBlockInfo wb;
    int block_index = -1;
    for (size_t i = 0; i < segment->message_capacity_; i++) {
      if (segment->arenas_[i].get() == arena_ptr) {
        block_index = i;
        break;
      }
    }
    if (block_index == -1) {
      return nullptr;
    }
    wb.block_index_ = block_index;
    ResetMessageRelatedBlocks(wrapper);
    this->AddMessageRelatedBlock(wrapper, block_index);
    SetMessageAddressOffset(
        wrapper, reinterpret_cast<uint64_t>(input_msg) -
                     reinterpret_cast<uint64_t>(segment->GetShmAddress()));
    msg_output = reinterpret_cast<void*>(
        const_cast<google::protobuf::Message*>(input_msg));
    segment->ReleaseWrittenBlock(wb);
  }

  return msg_output;
}

void* ProtobufArenaManager::GetMessage(message::ArenaMessageWrapper* wrapper) {
  auto segment = GetSegment(GetMessageChannelId(wrapper));
  if (!segment) {
    return nullptr;
  }

  auto address = reinterpret_cast<uint64_t>(segment->GetShmAddress()) +
                 GetMessageAddressOffset(wrapper);

  return reinterpret_cast<void*>(address);
}

bool ProtobufArenaManager::Enable() {
  if (init_) {
    return true;
  }

  // do something

  init_ = true;
  return true;
}

bool ProtobufArenaManager::EnableSegment(uint64_t channel_id) {
  if (segments_.find(channel_id) != segments_.end()) {
    if (arena_buffer_callbacks_.find(channel_id) !=
        arena_buffer_callbacks_.end()) {
      arena_buffer_callbacks_[channel_id]();
    }
    return true;
  }

  // uint64_t asociated_channel_id = channel_id + 1;
  // auto segment = SegmentFactory::CreateSegment(asociated_channel_id);
  // segment->InitOnly(10 * 1024);
  auto cyber_config = apollo::cyber::common::GlobalData::Instance()->Config();
  if (!cyber_config.has_transport_conf()) {
    return false;
  }
  if (!cyber_config.transport_conf().has_shm_conf()) {
    return false;
  }
  if (!cyber_config.transport_conf().shm_conf().has_arena_shm_conf()) {
    return false;
  }
  if (!cyber::common::GlobalData::Instance()->IsChannelEnableArenaShm(
          channel_id)) {
    return false;
  }
  auto arena_conf =
      cyber::common::GlobalData::Instance()->GetChannelArenaConf(channel_id);
  auto segment_shm_address = address_allocator_->Allocate(channel_id);
  auto segment = std::make_shared<ArenaSegment>(
      channel_id, arena_conf.max_msg_size(), arena_conf.max_pool_size(),
      reinterpret_cast<void*>(segment_shm_address));
  segments_[channel_id] = segment;
  if (arena_buffer_callbacks_.find(channel_id) !=
      arena_buffer_callbacks_.end()) {
    arena_buffer_callbacks_[channel_id]();
  }
  return true;
}

bool ProtobufArenaManager::Destroy() {
  if (!init_) {
    return true;
  }

  for (auto& segment : segments_) {
    address_allocator_->Deallocate(segment.first);
  }
  for (auto& buffer : non_arena_buffers_) {
    delete buffer.second;
  }
  segments_.clear();

  init_ = false;
  return true;
}

void ProtobufArenaManager::SetMessageChannelId(
    message::ArenaMessageWrapper* wrapper, uint64_t channel_id) {
  wrapper->GetExtended<ExtendedStruct>()->meta_.channel_id_ = channel_id;
}

uint64_t ProtobufArenaManager::GetMessageChannelId(
    message::ArenaMessageWrapper* wrapper) {
  return wrapper->GetExtended<ExtendedStruct>()->meta_.channel_id_;
}

void ProtobufArenaManager::SetMessageAddressOffset(
    message::ArenaMessageWrapper* wrapper, uint64_t address_offset) {
  wrapper->GetExtended<ExtendedStruct>()->meta_.address_offset_ =
      address_offset;
}

uint64_t ProtobufArenaManager::GetMessageAddressOffset(
    message::ArenaMessageWrapper* wrapper) {
  return wrapper->GetExtended<ExtendedStruct>()->meta_.address_offset_;
}

std::vector<uint64_t> ProtobufArenaManager::GetMessageRelatedBlocks(
    message::ArenaMessageWrapper* wrapper) {
  std::vector<uint64_t> related_blocks;
  auto extended = wrapper->GetExtended<ExtendedStruct>();
  for (uint64_t i = 0; i < extended->meta_.related_blocks_size_; ++i) {
    related_blocks.push_back(extended->meta_.related_blocks_[i]);
  }
  return related_blocks;
}

void ProtobufArenaManager::ResetMessageRelatedBlocks(
    message::ArenaMessageWrapper* wrapper) {
  auto extended = wrapper->GetExtended<ExtendedStruct>();
  extended->meta_.related_blocks_size_ = 0;
  // memset(extended->meta_.related_blocks_, 0,
  //        sizeof(extended->meta_.related_blocks_));
}

void ProtobufArenaManager::AddMessageRelatedBlock(
    message::ArenaMessageWrapper* wrapper, uint64_t block_index) {
  auto extended = wrapper->GetExtended<ExtendedStruct>();
  if (extended->meta_.related_blocks_size_ >=
      sizeof(extended->meta_.related_blocks_) / sizeof(uint64_t)) {
    return;
  }
  extended->meta_.related_blocks_[extended->meta_.related_blocks_size_++] =
      block_index;
}

ProtobufArenaManager::ArenaAllocCallback ProtobufArenaManager::arena_alloc_cb_ =
    nullptr;

void* ProtobufArenaManager::ArenaAlloc(uint64_t size) {
  return arena_alloc_cb_ ? arena_alloc_cb_(size) : nullptr;
}

void ProtobufArenaManager::ArenaDealloc(void* addr, uint64_t size) {}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
