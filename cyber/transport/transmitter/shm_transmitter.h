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

#ifndef CYBER_TRANSPORT_TRANSMITTER_SHM_TRANSMITTER_H_
#define CYBER_TRANSPORT_TRANSMITTER_SHM_TRANSMITTER_H_

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/util.h"
#include "cyber/message/arena_message_wrapper.h"
#include "cyber/message/message_traits.h"
#include "cyber/statistics/statistics.h"
#include "cyber/transport/shm/notifier_factory.h"
#include "cyber/transport/shm/protobuf_arena_manager.h"
#include "cyber/transport/shm/readable_info.h"
#include "cyber/transport/shm/segment_factory.h"
#include "cyber/transport/transmitter/transmitter.h"

namespace apollo {
namespace cyber {
namespace transport {

template <typename T, typename U>
struct type_check : std::is_same<typename std::decay<T>::type, U>::type {};

template <typename M>
class ShmTransmitter : public Transmitter<M> {
 public:
  using MessagePtr = std::shared_ptr<M>;

  explicit ShmTransmitter(const RoleAttributes& attr);
  virtual ~ShmTransmitter();

  void Enable() override;
  void Disable() override;

  void Enable(const RoleAttributes& opposite_attr);
  void Disable(const RoleAttributes& opposite_attr);

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

  bool AcquireMessage(std::shared_ptr<M>& msg);

 private:
  bool Transmit(const M& msg, const MessageInfo& msg_info);

  SegmentPtr segment_;
  uint64_t channel_id_;
  uint64_t host_id_;
  NotifierPtr notifier_;
  std::atomic<int> serialized_receiver_count_;
  std::atomic<int> arena_receiver_count_;
  bool arena_transmit_;
};

template <typename M>
bool ShmTransmitter<M>::AcquireMessage(std::shared_ptr<M>& msg) {
  if (this->enabled_) {
    auto msg_o = msg.get();
    auto arena_manager = ProtobufArenaManager::Instance();
    if (!arena_manager->Enable() ||
        !arena_manager->EnableSegment(this->attr_.channel_id())) {
      ADEBUG << "arena manager enable failed.";
      return false;
    }
    arena_manager->AcquireArenaMessage(channel_id_, msg);
    if (msg.get() != msg_o) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

template <typename M>
ShmTransmitter<M>::ShmTransmitter(const RoleAttributes& attr)
    : Transmitter<M>(attr),
      segment_(nullptr),
      channel_id_(attr.channel_id()),
      notifier_(nullptr),
      serialized_receiver_count_(0),
      arena_receiver_count_(0) {
  host_id_ = common::Hash(attr.host_ip());
  arena_transmit_ = common::GlobalData::Instance()->IsChannelEnableArenaShm(
                        this->attr_.channel_id()) &&
                    !type_check<M, message::RawMessage>::value &&
                    !type_check<M, message::PyMessageWrap>::value;
}

template <typename M>
ShmTransmitter<M>::~ShmTransmitter() {
  Disable();
}

template <typename M>
void ShmTransmitter<M>::Enable(const RoleAttributes& opposite_attr) {
  if (arena_transmit_) {
    if (opposite_attr.message_type() ==
            message::MessageType<message::RawMessage>() ||
        opposite_attr.message_type() ==
            message::MessageType<message::PyMessageWrap>()) {
      serialized_receiver_count_.fetch_add(1);
    } else {
      arena_receiver_count_.fetch_add(1);
    }
  } else {
    serialized_receiver_count_.fetch_add(1);
  }
  if (!this->enabled_) {
    this->Enable();
  }
}

template <typename M>
void ShmTransmitter<M>::Disable(const RoleAttributes& opposite_attr) {
  if (this->enabled_) {
    if (arena_transmit_) {
      if (opposite_attr.message_type() ==
              message::MessageType<message::RawMessage>() ||
          opposite_attr.message_type() ==
              message::MessageType<message::PyMessageWrap>()) {
        serialized_receiver_count_.fetch_sub(1);
      } else {
        arena_receiver_count_.fetch_sub(1);
      }
      if (serialized_receiver_count_.load() <= 0 &&
          arena_receiver_count_.load() <= 0) {
        this->Disable();
      }
    } else {
      serialized_receiver_count_.fetch_sub(1);
      if (serialized_receiver_count_.load() <= 0) {
        this->Disable();
      }
    }
  }
}

template <typename M>
void ShmTransmitter<M>::Enable() {
  if (this->enabled_) {
    return;
  }

  if (serialized_receiver_count_.load() == 0 &&
      arena_receiver_count_.load() == 0) {
    AERROR << "please enable shm transmitter by passing role attr.";
    return;
  }

  if (arena_transmit_) {
    auto arena_manager = ProtobufArenaManager::Instance();
    if (!arena_manager->Enable() ||
        !arena_manager->EnableSegment(this->attr_.channel_id())) {
      AERROR << "arena manager enable failed.";
      return;
    }
  }

  segment_ = SegmentFactory::CreateSegment(channel_id_);
  notifier_ = NotifierFactory::CreateNotifier();
  this->enabled_ = true;
}

template <typename M>
void ShmTransmitter<M>::Disable() {
  if (this->enabled_) {
    segment_ = nullptr;
    notifier_ = nullptr;
    this->enabled_ = false;
  }
}

template <typename M>
bool ShmTransmitter<M>::Transmit(const MessagePtr& msg,
                                 const MessageInfo& msg_info) {
  return Transmit(*msg, msg_info);
}

template <typename M>
bool ShmTransmitter<M>::Transmit(const M& msg, const MessageInfo& msg_info) {
  if (!this->enabled_) {
    ADEBUG << "not enable.";
    return false;
  }

  ReadableInfo readable_info;
  WritableBlock arena_wb;
  WritableBlock wb;

  readable_info.set_host_id(host_id_);
  readable_info.set_channel_id(channel_id_);
  readable_info.set_arena_block_index(-1);
  readable_info.set_block_index(-1);

  if (arena_transmit_) {
    // std::size_t msg_size = sizeof(message::ArenaMessageWrapper);
    std::size_t msg_size = 1024;
    if (!segment_->AcquireArenaBlockToWrite(msg_size, &arena_wb)) {
      AERROR << "acquire block failed.";
      return false;
    }

    ADEBUG << "arena block index: " << arena_wb.index;
    auto arena_manager = ProtobufArenaManager::Instance();
    auto msg_wrapper = arena_manager->CreateMessageWrapper();
    arena_manager->SetMessageChannelId(msg_wrapper.get(), channel_id_);
    M* msg_p;
    // arena_manager->CreateMessage(msg_wrapper.get(), msg);
    if (!message::SerializeToArenaMessageWrapper(msg, msg_wrapper.get(),
                                                 &msg_p)) {
      AERROR << "serialize to arena message wrapper failed.";
      segment_->ReleaseArenaWrittenBlock(arena_wb);
      return false;
    }
    auto segment = arena_manager->GetSegment(channel_id_);
    // auto msg_n =
    // std::shared_ptr<M>(
    // msg_p, [arena_manager, segment, msg_wrapper](M* p) {
    // auto related_blocks =
    //     arena_manager->GetMessageRelatedBlocks(msg_wrapper.get());
    // for (auto block_index : related_blocks) {
    //   // segment->ReleaseBlockForWriteByIndex(block_index);
    //   segment->RemoveBlockWriteLock(block_index);
    // }
    // });
    // for (auto block_index :
    //      arena_manager->GetMessageRelatedBlocks(msg_wrapper.get())) {
    //   segment->LockBlockForWriteByIndex(block_index);
    // }
    memcpy(arena_wb.buf, msg_wrapper->GetData(), msg_size);
    arena_wb.block->set_msg_size(msg_size);

    char* msg_info_addr = reinterpret_cast<char*>(arena_wb.buf) + msg_size;
    if (!msg_info.SerializeTo(msg_info_addr, MessageInfo::kSize)) {
      AERROR << "serialize message info failed.";
      segment_->ReleaseArenaWrittenBlock(arena_wb);
      return false;
    }
    arena_wb.block->set_msg_info_size(MessageInfo::kSize);
    readable_info.set_arena_block_index(arena_wb.index);
    if (serialized_receiver_count_.load() > 0) {
      std::size_t msg_size = message::ByteSize(msg);
      if (!segment_->AcquireBlockToWrite(msg_size, &wb)) {
        AERROR << "acquire block failed.";
        return false;
      }

      ADEBUG << "block index: " << wb.index;
      if (!message::SerializeToArray(msg, wb.buf, static_cast<int>(msg_size))) {
        AERROR << "serialize to array failed.";
        segment_->ReleaseWrittenBlock(wb);
        return false;
      }
      wb.block->set_msg_size(msg_size);

      char* msg_info_addr = reinterpret_cast<char*>(wb.buf) + msg_size;
      if (!msg_info.SerializeTo(msg_info_addr, MessageInfo::kSize)) {
        AERROR << "serialize message info failed.";
        segment_->ReleaseWrittenBlock(wb);
        return false;
      }
      wb.block->set_msg_info_size(MessageInfo::kSize);
      segment_->ReleaseWrittenBlock(wb);
      segment_->ReleaseArenaWrittenBlock(arena_wb);
      readable_info.set_block_index(wb.index);
    } else {
      segment_->ReleaseArenaWrittenBlock(arena_wb);
    }
  } else {
    std::size_t msg_size = message::ByteSize(msg);
    if (!segment_->AcquireBlockToWrite(msg_size, &wb)) {
      AERROR << "acquire block failed.";
      return false;
    }

    ADEBUG << "block index: " << wb.index;
    if (!message::SerializeToArray(msg, wb.buf, static_cast<int>(msg_size))) {
      AERROR << "serialize to array failed.";
      segment_->ReleaseWrittenBlock(wb);
      return false;
    }
    wb.block->set_msg_size(msg_size);

    char* msg_info_addr = reinterpret_cast<char*>(wb.buf) + msg_size;
    if (!msg_info.SerializeTo(msg_info_addr, MessageInfo::kSize)) {
      AERROR << "serialize message info failed.";
      segment_->ReleaseWrittenBlock(wb);
      return false;
    }
    wb.block->set_msg_info_size(MessageInfo::kSize);
    segment_->ReleaseWrittenBlock(wb);
    readable_info.set_block_index(wb.index);
  }

  ADEBUG << "Writing sharedmem message: "
         << common::GlobalData::GetChannelById(channel_id_)
         << " to normal block: " << readable_info.block_index()
         << " to arena block: " << readable_info.arena_block_index();
  return notifier_->Notify(readable_info);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_TRANSMITTER_SHM_TRANSMITTER_H_
