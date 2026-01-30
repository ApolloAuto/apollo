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

#ifndef CYBER_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_
#define CYBER_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_

#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/message/arena_message_wrapper.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/raw_message.h"
#include "cyber/statistics/statistics.h"
#include "cyber/time/time.h"
#include "cyber/transport/dispatcher/dispatcher.h"
#include "cyber/transport/shm/notifier_factory.h"
#include "cyber/transport/shm/protobuf_arena_manager.h"
#include "cyber/transport/shm/segment_factory.h"

namespace apollo {
namespace cyber {
namespace transport {

class ShmDispatcher;
using ShmDispatcherPtr = ShmDispatcher*;
using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;

class ShmDispatcher : public Dispatcher {
 public:
  // key: channel_id
  using SegmentContainer = std::unordered_map<uint64_t, SegmentPtr>;

  virtual ~ShmDispatcher();

  void Shutdown() override;

  template <typename MessageT>
  void AddListener(const RoleAttributes& self_attr,
                   const MessageListener<MessageT>& listener);

  template <typename MessageT>
  void AddListener(const RoleAttributes& self_attr,
                   const RoleAttributes& opposite_attr,
                   const MessageListener<MessageT>& listener);

  template <typename MessageT>
  void AddArenaListener(const RoleAttributes& self_attr,
                        const MessageListener<MessageT>& listener);

  template <typename MessageT>
  void AddArenaListener(const RoleAttributes& self_attr,
                        const RoleAttributes& opposite_attr,
                        const MessageListener<MessageT>& listener);

 private:
  void AddSegment(const RoleAttributes& self_attr);
  void ReadMessage(uint64_t channel_id, uint32_t block_index);
  void OnMessage(uint64_t channel_id, const std::shared_ptr<ReadableBlock>& rb,
                 const MessageInfo& msg_info);
  void ReadArenaMessage(uint64_t channel_id, uint32_t arena_block_index);
  void OnArenaMessage(uint64_t channel_id,
                      const std::shared_ptr<ReadableBlock>& rb,
                      const MessageInfo& msg_info);
  void ThreadFunc();
  bool Init();

  uint64_t host_id_;
  SegmentContainer segments_;
  std::unordered_map<uint64_t, uint32_t> previous_indexes_;
  std::unordered_map<uint64_t, uint32_t> arena_previous_indexes_;
  AtomicHashMap<uint64_t, ListenerHandlerBasePtr> arena_msg_listeners_;
  AtomicRWLock segments_lock_;
  std::thread thread_;
  NotifierPtr notifier_;

  DECLARE_SINGLETON(ShmDispatcher)
};

template <typename MessageT>
void ShmDispatcher::AddArenaListener(
    const RoleAttributes& self_attr,
    const MessageListener<MessageT>& listener) {
  if (is_shutdown_.load()) {
    return;
  }
  uint64_t channel_id = self_attr.channel_id();

  std::shared_ptr<ListenerHandler<MessageT>> handler;
  ListenerHandlerBasePtr* handler_base = nullptr;
  if (arena_msg_listeners_.Get(channel_id, &handler_base)) {
    handler =
        std::dynamic_pointer_cast<ListenerHandler<MessageT>>(*handler_base);
    if (handler == nullptr) {
      AERROR << "please ensure that readers with the same channel["
             << self_attr.channel_name()
             << "] in the same process have the same message type";
      return;
    }
  } else {
    ADEBUG << "new reader for channel:"
           << GlobalData::GetChannelById(channel_id);
    handler.reset(new ListenerHandler<MessageT>());
    arena_msg_listeners_.Set(channel_id, handler);
  }
  handler->Connect(self_attr.id(), listener);
}

template <typename MessageT>
void ShmDispatcher::AddArenaListener(
    const RoleAttributes& self_attr, const RoleAttributes& opposite_attr,
    const MessageListener<MessageT>& listener) {
  if (is_shutdown_.load()) {
    return;
  }
  uint64_t channel_id = self_attr.channel_id();
  std::shared_ptr<ListenerHandler<MessageT>> handler;
  ListenerHandlerBasePtr* handler_base = nullptr;
  if (arena_msg_listeners_.Get(channel_id, &handler_base)) {
    handler =
        std::dynamic_pointer_cast<ListenerHandler<MessageT>>(*handler_base);
    if (handler == nullptr) {
      AERROR << "please ensuore that readers with the same channel["
             << self_attr.channel_name()
             << "] in the same process have the same message type";
      return;
    }
  } else {
    ADEBUG << "new reader for channel:"
           << GlobalData::GetChannelById(channel_id);
    handler.reset(new ListenerHandler<MessageT>());
    arena_msg_listeners_.Set(channel_id, handler);
  }
  handler->Connect(self_attr.id(), listener);
}

template <typename MessageT>
void ShmDispatcher::AddListener(const RoleAttributes& self_attr,
                                const MessageListener<MessageT>& listener) {
  // FIXME: make it more clean
  if (cyber::common::GlobalData::Instance()->IsChannelEnableArenaShm(
          self_attr.channel_id()) &&
      self_attr.message_type() != message::MessageType<message::RawMessage>() &&
      self_attr.message_type() !=
          message::MessageType<message::PyMessageWrap>()) {
    auto listener_adapter = [listener, self_attr](
                                const std::shared_ptr<ReadableBlock>& rb,
                                const MessageInfo& msg_info) {
      auto msg = std::make_shared<MessageT>();
      // TODO(ALL): read config from msg_info
      auto arena_manager = ProtobufArenaManager::Instance();
      auto msg_wrapper = arena_manager->CreateMessageWrapper();
      memcpy(msg_wrapper->GetData(), rb->buf, 1024);
      MessageT* msg_p;
      if (!message::ParseFromArenaMessageWrapper(msg_wrapper.get(), msg.get(),
                                                 &msg_p)) {
        AERROR << "ParseFromArenaMessageWrapper failed";
      }
      // msg->CopyFrom(*msg_p);
      // msg = arena_manager->LoadMessage<MessageT>(msg_wrapper.get())
      auto segment = arena_manager->GetSegment(self_attr.channel_id());
      auto msg_addr = reinterpret_cast<uint64_t>(msg_p);
      msg.reset(reinterpret_cast<MessageT*>(msg_addr),
                [arena_manager, segment, msg_wrapper](MessageT* p) {
                  // fprintf(stderr, "msg deleter invoked\n");
                  // auto related_blocks =
                  //     arena_manager->GetMessageRelatedBlocks(msg_wrapper.get());
                  // for (auto block_index : related_blocks) {
                  //   // segment->ReleaseBlockForReadByIndex(block_index);
                  //   segment->RemoveBlockReadLock(block_index);
                  // }
                });
      for (auto block_index :
           arena_manager->GetMessageRelatedBlocks(msg_wrapper.get())) {
        segment->AddBlockReadLock(block_index);
      }

      auto send_time = msg_info.send_time();
      auto msg_seq_num = msg_info.msg_seq_num();

      statistics::Statistics::Instance()->AddRecvCount(self_attr,
                                                       msg_info.msg_seq_num());
      statistics::Statistics::Instance()->SetTotalMsgsStatus(self_attr,
                                                             msg_seq_num);

      auto recv_time = Time::Now().ToNanosecond();

      // sampling in microsecond
      auto tran_diff = (recv_time - send_time) / 1000;
      if (tran_diff > 0) {
        // sample transport latency in microsecond
        statistics::Statistics::Instance()->SamplingTranLatency<uint64_t>(
            self_attr, tran_diff);
      }
      statistics::Statistics::Instance()->SetProcStatus(self_attr,
                                                        recv_time / 1000);
      listener(msg, msg_info);
      auto related_blocks =
          arena_manager->GetMessageRelatedBlocks(msg_wrapper.get());
      for (auto block_index : related_blocks) {
        // segment->ReleaseBlockForReadByIndex(block_index);
        segment->RemoveBlockReadLock(block_index);
      }
    };

    AddArenaListener<ReadableBlock>(self_attr, listener_adapter);
  } else {
    auto listener_adapter = [listener, self_attr](
                                const std::shared_ptr<ReadableBlock>& rb,
                                const MessageInfo& msg_info) {
      auto msg = std::make_shared<MessageT>();
      // TODO(ALL): read config from msg_info
      RETURN_IF(!message::ParseFromArray(
          rb->buf, static_cast<int>(rb->block->msg_size()), msg.get()));

      auto send_time = msg_info.send_time();
      auto msg_seq_num = msg_info.msg_seq_num();

      statistics::Statistics::Instance()->AddRecvCount(self_attr,
                                                       msg_info.msg_seq_num());
      statistics::Statistics::Instance()->SetTotalMsgsStatus(self_attr,
                                                             msg_seq_num);

      auto recv_time = Time::Now().ToNanosecond();

      // sampling in microsecond
      auto tran_diff = (recv_time - send_time) / 1000;
      if (tran_diff > 0) {
        // sample transport latency in microsecond
        statistics::Statistics::Instance()->SamplingTranLatency<uint64_t>(
            self_attr, tran_diff);
      }
      statistics::Statistics::Instance()->SetProcStatus(self_attr,
                                                        recv_time / 1000);
      listener(msg, msg_info);
    };

    Dispatcher::AddListener<ReadableBlock>(self_attr, listener_adapter);
  }
  AddSegment(self_attr);
}

template <typename MessageT>
void ShmDispatcher::AddListener(const RoleAttributes& self_attr,
                                const RoleAttributes& opposite_attr,
                                const MessageListener<MessageT>& listener) {
  // FIXME: make it more clean
  if (cyber::common::GlobalData::Instance()->IsChannelEnableArenaShm(
          self_attr.channel_id()) &&
      self_attr.message_type() != message::MessageType<message::RawMessage>() &&
      self_attr.message_type() !=
          message::MessageType<message::PyMessageWrap>()) {
    auto listener_adapter = [listener, self_attr](
                                const std::shared_ptr<ReadableBlock>& rb,
                                const MessageInfo& msg_info) {
      auto msg = std::make_shared<MessageT>();
      auto arena_manager = ProtobufArenaManager::Instance();
      auto msg_wrapper = arena_manager->CreateMessageWrapper();
      memcpy(msg_wrapper->GetData(), rb->buf, 1024);
      MessageT* msg_p;
      if (!message::ParseFromArenaMessageWrapper(msg_wrapper.get(), msg.get(),
                                                 &msg_p)) {
        AERROR << "ParseFromArenaMessageWrapper failed";
      }
      // msg->CopyFrom(*msg_p);
      // msg = arena_manager->LoadMessage<MessageT>(msg_wrapper.get())
      auto segment = arena_manager->GetSegment(self_attr.channel_id());
      auto msg_addr = reinterpret_cast<uint64_t>(msg_p);
      msg.reset(reinterpret_cast<MessageT*>(msg_addr),
                [arena_manager, segment, msg_wrapper](MessageT* p) {
                  // fprintf(stderr, "msg deleter invoked\n");
                  // auto related_blocks =
                  //     arena_manager->GetMessageRelatedBlocks(msg_wrapper.get());
                  // for (auto block_index : related_blocks) {
                  //   // segment->ReleaseBlockForReadByIndex(block_index);
                  //   segment->RemoveBlockReadLock(block_index);
                  // }
                });
      for (auto block_index :
           arena_manager->GetMessageRelatedBlocks(msg_wrapper.get())) {
        segment->AddBlockReadLock(block_index);
      }

      auto send_time = msg_info.send_time();
      auto msg_seq_num = msg_info.msg_seq_num();

      statistics::Statistics::Instance()->AddRecvCount(self_attr,
                                                       msg_info.msg_seq_num());
      statistics::Statistics::Instance()->SetTotalMsgsStatus(self_attr,
                                                             msg_seq_num);

      auto recv_time = Time::Now().ToNanosecond();

      // sampling in microsecond
      auto tran_diff = (recv_time - send_time) / 1000;
      if (tran_diff > 0) {
        statistics::Statistics::Instance()->SamplingTranLatency<uint64_t>(
            self_attr, tran_diff);
      }
      statistics::Statistics::Instance()->SetProcStatus(self_attr,
                                                        recv_time / 1000);

      listener(msg, msg_info);
      auto related_blocks =
          arena_manager->GetMessageRelatedBlocks(msg_wrapper.get());
      for (auto block_index : related_blocks) {
        // segment->ReleaseBlockForReadByIndex(block_index);
        segment->RemoveBlockReadLock(block_index);
      }
    };

    AddArenaListener<ReadableBlock>(self_attr, opposite_attr, listener_adapter);
  } else {
    auto listener_adapter = [listener, self_attr](
                                const std::shared_ptr<ReadableBlock>& rb,
                                const MessageInfo& msg_info) {
      auto msg = std::make_shared<MessageT>();
      RETURN_IF(!message::ParseFromArray(
          rb->buf, static_cast<int>(rb->block->msg_size()), msg.get()));

      auto send_time = msg_info.send_time();
      auto msg_seq_num = msg_info.msg_seq_num();

      statistics::Statistics::Instance()->AddRecvCount(self_attr,
                                                       msg_info.msg_seq_num());
      statistics::Statistics::Instance()->SetTotalMsgsStatus(self_attr,
                                                             msg_seq_num);

      auto recv_time = Time::Now().ToNanosecond();

      // sampling in microsecond
      auto tran_diff = (recv_time - send_time) / 1000;
      if (tran_diff > 0) {
        statistics::Statistics::Instance()->SamplingTranLatency<uint64_t>(
            self_attr, tran_diff);
      }
      statistics::Statistics::Instance()->SetProcStatus(self_attr,
                                                        recv_time / 1000);

      listener(msg, msg_info);
    };

    Dispatcher::AddListener<ReadableBlock>(self_attr, opposite_attr,
                                           listener_adapter);
  }
  AddSegment(self_attr);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_
