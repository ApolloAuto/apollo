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
#ifndef CYBER_TRANSPORT_SHM_PROTOBUF_ARENA_MANAGER_H_
#define CYBER_TRANSPORT_SHM_PROTOBUF_ARENA_MANAGER_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include <google/protobuf/arena.h>

#include "cyber/base/arena_queue.h"
#include "cyber/base/pthread_rw_lock.h"
#include "cyber/common/global_data.h"
#include "cyber/common/macros.h"
#include "cyber/message/arena_manager_base.h"
#include "cyber/message/arena_message_wrapper.h"
#include "cyber/transport/shm/arena_address_allocator.h"
#include "cyber/transport/shm/segment.h"

namespace apollo {
namespace cyber {
namespace transport {

union ArenaSegmentState {
  struct {
    std::atomic<uint64_t> ref_count_;
    std::atomic<bool> auto_extended_;
    std::atomic<uint64_t> message_size_;
    std::atomic<uint64_t> block_num_;
    std::atomic<uint64_t> message_seq_;
    std::mutex mutex_;
  } struct_;
  uint8_t bytes_[128];
};

union ArenaSegmentBlockDescriptor {
  struct {
    uint64_t size_;
    std::atomic<uint64_t> writing_ref_count_;
    std::atomic<uint64_t> reading_ref_count_;
    base::PthreadRWLock read_write_mutex_;
  } struct_;
  uint8_t bytes_[128];
};

struct ArenaSegmentBlock {
  uint64_t size_;
  // std::atomic<uint64_t> writing_ref_count_;
  // std::atomic<uint64_t> reading_ref_count_;
  // base::PthreadRWLock read_write_mutex_;
  static const int32_t kRWLockFree;
  static const int32_t kWriteExclusive;
  static const int32_t kMaxTryLockTimes;
  std::atomic<int32_t> lock_num_ = {0};
};

struct ArenaSegmentBlockInfo {
  uint64_t block_index_;
  ArenaSegmentBlock* block_;
  void* block_buffer_address_;
};

class ArenaSegment {
 public:
  ArenaSegment();
  explicit ArenaSegment(uint64_t channel_id);
  ArenaSegment(uint64_t channel_id, void* base_address);
  ArenaSegment(uint64_t channel_id, uint64_t message_size, uint64_t block_num,
               void* base_address);
  ~ArenaSegment();

  bool Init(uint64_t message_size, uint64_t block_num);
  // bool Create(uint64_t message_size, uint64_t block_num);
  bool Open(uint64_t message_size, uint64_t block_num);
  bool OpenOrCreate(uint64_t message_size, uint64_t block_num);

  void* GetShmAddress();

  uint64_t GetNextWritableBlockIndex();
  bool AddBlockWriteLock(uint64_t block_index);
  void RemoveBlockWriteLock(uint64_t block_index);
  bool AddBlockReadLock(uint64_t block_index);
  void RemoveBlockReadLock(uint64_t block_index);

  bool AcquireBlockToWrite(uint64_t size, ArenaSegmentBlockInfo* block_info);
  void ReleaseWrittenBlock(const ArenaSegmentBlockInfo& block_info);

  bool AcquireBlockToRead(ArenaSegmentBlockInfo* block_info);
  void ReleaseReadBlock(const ArenaSegmentBlockInfo& block_info);

  // uint64_t GetCapicity();

 public:
  ArenaSegmentState* state_;
  ArenaSegmentBlock* blocks_;
  std::vector<std::shared_ptr<google::protobuf::Arena>> arenas_;
  std::vector<uint64_t> arena_block_address_;
  uint64_t channel_id_;
  uint64_t key_id_;
  void* base_address_;
  void* shm_address_;
  std::shared_ptr<google::protobuf::Arena> shared_buffer_arena_;
  void* arena_buffer_address_ = nullptr;

  uint64_t message_capacity_;
};

union ExtendedStruct {
  struct {
    uint64_t channel_id_;
    uint64_t address_offset_;
    uint64_t related_blocks_[4];
    uint64_t related_blocks_size_ = 0;
  } meta_;
  uint8_t bytes_[256];
};

union ProtobufArenaManagerMeta {
  struct {
    uint64_t version_;
    uint64_t base_address_;
    uint64_t segment_size_;
    uint64_t block_size_;
    uint64_t block_num_;
    uint64_t message_size_;
    uint64_t message_num_;
    uint64_t message_seq_;
    uint64_t extended_struct_size_;
    uint64_t extended_struct_num_;
    uint64_t extended_struct_seq_;
  } struct_;
  uint8_t bytes_[256];
};

class ProtobufArenaManager : public message::ArenaManagerBase {
 public:
  using ArenaAllocCallback = std::function<void*(uint64_t)>;

  ~ProtobufArenaManager();

  uint64_t GetBaseAddress(const message::ArenaMessageWrapper* wrapper) override;

  bool Enable();
  bool EnableSegment(uint64_t channel_id);
  bool Destroy();
  // bool Shutdown();

  void SetMessageChannelId(message::ArenaMessageWrapper* wrapper,
                           uint64_t channel_id);
  uint64_t GetMessageChannelId(message::ArenaMessageWrapper* wrapper);
  void SetMessageAddressOffset(message::ArenaMessageWrapper* wrapper,
                               uint64_t offset);
  uint64_t GetMessageAddressOffset(message::ArenaMessageWrapper* wrapper);
  std::vector<uint64_t> GetMessageRelatedBlocks(
      message::ArenaMessageWrapper* wrapper);
  void ResetMessageRelatedBlocks(message::ArenaMessageWrapper* wrapper);
  void AddMessageRelatedBlock(message::ArenaMessageWrapper* wrapper,
                              uint64_t block_index);

  std::shared_ptr<ArenaSegment> GetSegment(uint64_t channel_id);

  void* SetMessage(message::ArenaMessageWrapper* wrapper,
                   const void* message) override;
  void* GetMessage(message::ArenaMessageWrapper* wrapper) override;

  void* GetAvailableBuffer(uint64_t channel_id) {
    auto segment = this->GetSegment(channel_id);
    if (!segment) {
      if (non_arena_buffers_.find(channel_id) == non_arena_buffers_.end()) {
        return nullptr;
      }
      return non_arena_buffers_[channel_id];
    }
    if (segment->arena_buffer_address_ != nullptr) {
      return segment->arena_buffer_address_;
    }
    return non_arena_buffers_[channel_id];
  }

  template <typename T>
  bool RegisterQueue(uint64_t channel_id, uint64_t size) {
    if (non_arena_buffers_.find(channel_id) == non_arena_buffers_.end() ||
        arena_buffer_callbacks_.find(channel_id) ==
            arena_buffer_callbacks_.end()) {
      auto non_arena_buffer_ptr = new apollo::cyber::base::ArenaQueue<T>();
      non_arena_buffer_ptr->Init(size);
      non_arena_buffers_[channel_id] = non_arena_buffer_ptr;
      arena_buffer_callbacks_[channel_id] = [this, channel_id, size]() {
        auto segment = this->GetSegment(channel_id);
        if (!segment) {
          ADEBUG << "channel id '" << channel_id << "' not enable";
          ADEBUG << "fallback to use nomarl queue";
          return;
        }
        if (segment->shared_buffer_arena_ == nullptr) {
          ADEBUG << "Not enable arena shared buffer in channel id '"
                 << channel_id << "'";
          ADEBUG << "fallback to use nomarl queue";
          return;
        }
        if (segment->arena_buffer_address_ == nullptr) {
          auto ptr = google::protobuf::Arena::Create<base::ArenaQueue<T>>(
              segment->shared_buffer_arena_.get());
          ptr->Init(size, segment->shared_buffer_arena_.get());
          segment->arena_buffer_address_ = reinterpret_cast<void*>(ptr);
        }
      };
    }
    // try enable arena buffer
    auto segment = GetSegment(channel_id);
    if (segment) {
      arena_buffer_callbacks_[channel_id]();
    }
    return true;
  }

  template <typename M,
            typename std::enable_if<
                google::protobuf::Arena::is_arena_constructable<M>::value,
                M>::type* = nullptr>
  void AcquireArenaMessage(uint64_t channel_id, std::shared_ptr<M>& ret_msg) {
    auto arena_conf =
        cyber::common::GlobalData::Instance()->GetChannelArenaConf(channel_id);
    google::protobuf::ArenaOptions options;
    options.start_block_size = arena_conf.max_msg_size();
    options.max_block_size = arena_conf.max_msg_size();

    auto segment = GetSegment(channel_id);
    if (!segment) {
      return;
    }

    ArenaSegmentBlockInfo wb;
    // TODO(All): size should be send to
    // AcquireBlockToWrite for dynamic adjust block size
    // auto size = input_msg->ByteSizeLong();
    uint64_t size = 0;
    if (!segment->AcquireBlockToWrite(size, &wb)) {
      return;
    }
    options.initial_block =
        reinterpret_cast<char*>(segment->arena_block_address_[wb.block_index_]);
    options.initial_block_size = segment->message_capacity_;
    if (segment->arenas_[wb.block_index_] != nullptr) {
      segment->arenas_[wb.block_index_] = nullptr;
    }
    segment->arenas_[wb.block_index_] =
        std::make_shared<google::protobuf::Arena>(options);

    // deconstructor do nothing to avoid proto
    // instance deconstructed before arena allocator
    ret_msg = std::shared_ptr<M>(
        google::protobuf::Arena::CreateMessage<M>(
            segment->arenas_[wb.block_index_].get()),
        [segment, wb](M* ptr) {
          int32_t lock_num = segment->blocks_[wb.block_index_].lock_num_.load();
          if (lock_num < ArenaSegmentBlock::kRWLockFree) {
            segment->ReleaseWrittenBlock(wb);
          }
        });
    return;
  }

  template <typename M,
            typename std::enable_if<
                !google::protobuf::Arena::is_arena_constructable<M>::value,
                M>::type* = nullptr>
  void AcquireArenaMessage(uint64_t channel_id, std::shared_ptr<M>& ret_msg) {
    return;
  }

 private:
  bool init_;
  std::unordered_map<uint64_t, std::shared_ptr<ArenaSegment>> segments_;
  std::unordered_map<uint64_t, void*> non_arena_buffers_;
  std::unordered_map<uint64_t, std::function<void()>> arena_buffer_callbacks_;
  std::mutex segments_mutex_;

  std::shared_ptr<ArenaAddressAllocator> address_allocator_;

  static ArenaAllocCallback arena_alloc_cb_;
  static void* ArenaAlloc(uint64_t size);
  static void ArenaDealloc(void* addr, uint64_t size);

  std::mutex arena_alloc_cb_mutex_;

  std::unordered_map<uint64_t, uint64_t> managed_wrappers_;

  DECLARE_SINGLETON(ProtobufArenaManager)
};

// template <typename MessageT>
// std::shared_ptr<MessageT> ProtobufArenaManager::CreateMessage(
//     message::ArenaMessageWrapper* wrapper, const MessageT& message) {}

// template <typename MessageT>
// std::shared_ptr<MessageT> ProtobufArenaManager::LoadMessage(
//     message::ArenaMessageWrapper* wrapper) {}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif
