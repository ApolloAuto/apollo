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

#ifndef CYBER_TRANSPORT_DISPATCHER_INTRA_DISPATCHER_H_
#define CYBER_TRANSPORT_DISPATCHER_INTRA_DISPATCHER_H_

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/raw_message.h"
#include "cyber/statistics/statistics.h"
#include "cyber/time/time.h"
#include "cyber/transport/dispatcher/dispatcher.h"

namespace apollo {
namespace cyber {
namespace transport {

class IntraDispatcher;
using IntraDispatcherPtr = IntraDispatcher*;
class ChannelChain;
using ChannelChainPtr = std::shared_ptr<ChannelChain>;
template <typename MessageT>
using MessageListener =
    std::function<void(const std::shared_ptr<MessageT>&, const MessageInfo&)>;

// use a channel chain to wrap specific ListenerHandler.
// If the message is MessageT, then we use pointer directly, or we first parse
// to a string, and use it to serialise to another message type.
class ChannelChain {
  using BaseHandlersType =
      std::map<uint64_t, std::map<std::string, ListenerHandlerBasePtr>>;

 public:
  template <typename MessageT>
  bool AddListener(uint64_t self_id, uint64_t channel_id,
                   const std::string& message_type,
                   const MessageListener<MessageT>& listener) {
    WriteLockGuard<base::AtomicRWLock> lg(rw_lock_);
    auto ret = GetHandler<MessageT>(channel_id, message_type, &handlers_);
    auto handler = ret.first;
    if (handler == nullptr) {
      AERROR << "get handler failed. channel: "
             << GlobalData::GetChannelById(channel_id)
             << ", message type: " << message::GetMessageName<MessageT>();
      return ret.second;
    }
    handler->Connect(self_id, listener);
    return ret.second;
  }

  template <typename MessageT>
  bool AddListener(uint64_t self_id, uint64_t oppo_id, uint64_t channel_id,
                   const std::string& message_type,
                   const MessageListener<MessageT>& listener) {
    WriteLockGuard<base::AtomicRWLock> lg(oppo_rw_lock_);
    if (oppo_handlers_.find(oppo_id) == oppo_handlers_.end()) {
      oppo_handlers_[oppo_id] = BaseHandlersType();
    }
    BaseHandlersType& handlers = oppo_handlers_[oppo_id];
    auto ret = GetHandler<MessageT>(channel_id, message_type, &handlers);
    auto handler = ret.first;
    if (handler == nullptr) {
      AERROR << "get handler failed. channel: "
             << GlobalData::GetChannelById(channel_id)
             << ", message type: " << message_type;
      return ret.second;
    }
    handler->Connect(self_id, oppo_id, listener);
    return ret.second;
  }

  template <typename MessageT>
  void RemoveListener(uint64_t self_id, uint64_t channel_id,
                      const std::string& message_type) {
    WriteLockGuard<base::AtomicRWLock> lg(rw_lock_);
    auto handler = RemoveHandler(channel_id, message_type, &handlers_);
    if (handler) {
      handler->Disconnect(self_id);
    }
  }

  template <typename MessageT>
  void RemoveListener(uint64_t self_id, uint64_t oppo_id, uint64_t channel_id,
                      const std::string& message_type) {
    WriteLockGuard<base::AtomicRWLock> lg(oppo_rw_lock_);
    if (oppo_handlers_.find(oppo_id) == oppo_handlers_.end()) {
      return;
    }
    BaseHandlersType& handlers = oppo_handlers_[oppo_id];
    auto handler = RemoveHandler(channel_id, message_type, &handlers);
    if (oppo_handlers_[oppo_id].empty()) {
      oppo_handlers_.erase(oppo_id);
    }
    if (handler) {
      handler->Disconnect(self_id, oppo_id);
    }
  }

  template <typename MessageT>
  void Run(uint64_t self_id, uint64_t channel_id,
           const std::string& message_type,
           const std::shared_ptr<MessageT>& message,
           const MessageInfo& message_info) {
    ReadLockGuard<base::AtomicRWLock> lg(rw_lock_);
    Run(channel_id, message_type, handlers_, message, message_info);
  }

  template <typename MessageT>
  void Run(uint64_t self_id, uint64_t oppo_id, uint64_t channel_id,
           const std::string& message_type,
           const std::shared_ptr<MessageT>& message,
           const MessageInfo& message_info) {
    ReadLockGuard<base::AtomicRWLock> lg(oppo_rw_lock_);
    if (oppo_handlers_.find(oppo_id) == oppo_handlers_.end()) {
      return;
    }
    BaseHandlersType& handlers = oppo_handlers_[oppo_id];
    Run(channel_id, message_type, handlers, message, message_info);
  }

 private:
  // NOTE: lock hold
  template <typename MessageT>
  std::pair<std::shared_ptr<ListenerHandler<MessageT>>, bool> GetHandler(
      uint64_t channel_id, const std::string& message_type,
      BaseHandlersType* handlers) {
    std::shared_ptr<ListenerHandler<MessageT>> handler;
    bool created = false;  // if the handler is created

    if (handlers->find(channel_id) == handlers->end()) {
      (*handlers)[channel_id] = std::map<std::string, ListenerHandlerBasePtr>();
    }

    if ((*handlers)[channel_id].find(message_type) ==
        (*handlers)[channel_id].end()) {
      ADEBUG << "Create new ListenerHandler for channel "
             << GlobalData::GetChannelById(channel_id)
             << ", message type: " << message_type;
      handler.reset(new ListenerHandler<MessageT>());
      (*handlers)[channel_id][message_type] = handler;
      created = true;
    } else {
      ADEBUG << "Find channel " << GlobalData::GetChannelById(channel_id)
             << "'s ListenerHandler, message type: " << message_type;
      handler = std::dynamic_pointer_cast<ListenerHandler<MessageT>>(
          (*handlers)[channel_id][message_type]);
    }

    return std::make_pair(handler, created);
  }

  // NOTE: Lock hold
  ListenerHandlerBasePtr RemoveHandler(int64_t channel_id,
                                       const std::string message_type,
                                       BaseHandlersType* handlers) {
    ListenerHandlerBasePtr handler_base;
    if (handlers->find(channel_id) != handlers->end()) {
      if ((*handlers)[channel_id].find(message_type) !=
          (*handlers)[channel_id].end()) {
        handler_base = (*handlers)[channel_id][message_type];
        ADEBUG << "remove " << GlobalData::GetChannelById(channel_id) << "'s "
               << message_type << " ListenerHandler";
        (*handlers)[channel_id].erase(message_type);
      }
      if ((*handlers)[channel_id].empty()) {
        ADEBUG << "remove " << GlobalData::GetChannelById(channel_id)
               << "'s all ListenerHandler";
        (*handlers).erase(channel_id);
      }
    }
    return handler_base;
  }

  template <typename MessageT>
  void Run(const uint64_t channel_id, const std::string& message_type,
           const BaseHandlersType& handlers,
           const std::shared_ptr<MessageT>& message,
           const MessageInfo& message_info) {
    const auto channel_handlers_itr = handlers.find(channel_id);
    if (channel_handlers_itr == handlers.end()) {
      AERROR << "Cant find channel " << GlobalData::GetChannelById(channel_id)
             << " in Chain";
      return;
    }
    const auto& channel_handlers = channel_handlers_itr->second;

    ADEBUG << GlobalData::GetChannelById(channel_id)
           << "'s chain run, size: " << channel_handlers.size()
           << ", message type: " << message_type;
    std::string msg;
    for (const auto& ele : channel_handlers) {
      auto handler_base = ele.second;
      if (message_type == ele.first) {
        ADEBUG << "Run handler for message type: " << ele.first << " directly";
        auto handler =
            std::static_pointer_cast<ListenerHandler<MessageT>>(handler_base);
        if (handler == nullptr) {
          continue;
        }
        handler->Run(message, message_info);
      } else {
        ADEBUG << "Run handler for message type: " << ele.first
               << " from string";
        if (msg.empty()) {
          auto msg_size = message::FullByteSize(*message);
          if (msg_size < 0) {
            AERROR << "Failed to get message size. channel["
                   << common::GlobalData::GetChannelById(channel_id) << "]";
            continue;
          }
          msg.resize(msg_size);
          if (!message::SerializeToHC(*message, const_cast<char*>(msg.data()),
                                      msg_size)) {
            AERROR << "Chain Serialize error for channel id: " << channel_id;
            msg.clear();
          }
        }
        if (!msg.empty()) {
          (handler_base)->RunFromString(msg, message_info);
        }
      }
    }
  }

  BaseHandlersType handlers_;
  base::AtomicRWLock rw_lock_;
  std::map<uint64_t, BaseHandlersType> oppo_handlers_;
  base::AtomicRWLock oppo_rw_lock_;
};

class IntraDispatcher : public Dispatcher {
 public:
  virtual ~IntraDispatcher();

  template <typename MessageT>
  void OnMessage(uint64_t channel_id, const std::shared_ptr<MessageT>& message,
                 const MessageInfo& message_info);

  template <typename MessageT>
  void AddListener(const RoleAttributes& self_attr,
                   const MessageListener<MessageT>& listener);

  template <typename MessageT>
  void AddListener(const RoleAttributes& self_attr,
                   const RoleAttributes& opposite_attr,
                   const MessageListener<MessageT>& listener);

  template <typename MessageT>
  void RemoveListener(const RoleAttributes& self_attr);

  template <typename MessageT>
  void RemoveListener(const RoleAttributes& self_attr,
                      const RoleAttributes& opposite_attr);

  DECLARE_SINGLETON(IntraDispatcher)

 private:
  template <typename MessageT>
  std::shared_ptr<ListenerHandler<MessageT>> GetHandler(uint64_t channel_id);

  ChannelChainPtr chain_;
};

template <typename MessageT>
void IntraDispatcher::OnMessage(uint64_t channel_id,
                                const std::shared_ptr<MessageT>& message,
                                const MessageInfo& message_info) {
  if (is_shutdown_.load()) {
    return;
  }
  ListenerHandlerBasePtr* handler_base = nullptr;
  ADEBUG << "intra on message, channel:"
         << common::GlobalData::GetChannelById(channel_id);
  if (msg_listeners_.Get(channel_id, &handler_base)) {
    auto handler =
        std::dynamic_pointer_cast<ListenerHandler<MessageT>>(*handler_base);
    if (handler) {
      handler->Run(message, message_info);
    } else {
      auto msg_size = message::FullByteSize(*message);
      if (msg_size < 0) {
        AERROR << "Failed to get message size. channel["
               << common::GlobalData::GetChannelById(channel_id) << "]";
        return;
      }
      std::string msg;
      msg.resize(msg_size);
      if (message::SerializeToHC(*message, const_cast<char*>(msg.data()),
                                 msg_size)) {
        (*handler_base)->RunFromString(msg, message_info);
      } else {
        AERROR << "Failed to serialize message. channel["
               << common::GlobalData::GetChannelById(channel_id) << "]";
      }
    }
  }
}

template <typename MessageT>
std::shared_ptr<ListenerHandler<MessageT>> IntraDispatcher::GetHandler(
    uint64_t channel_id) {
  std::shared_ptr<ListenerHandler<MessageT>> handler;
  ListenerHandlerBasePtr* handler_base = nullptr;

  if (msg_listeners_.Get(channel_id, &handler_base)) {
    handler =
        std::dynamic_pointer_cast<ListenerHandler<MessageT>>(*handler_base);
    if (handler == nullptr) {
      ADEBUG << "Find a new type for channel "
             << GlobalData::GetChannelById(channel_id) << " with type "
             << message::GetMessageName<MessageT>();
    }
  } else {
    ADEBUG << "Create new ListenerHandler for channel "
           << GlobalData::GetChannelById(channel_id) << " with type "
           << message::GetMessageName<MessageT>();
    handler.reset(new ListenerHandler<MessageT>());
    msg_listeners_.Set(channel_id, handler);
  }

  return handler;
}

template <typename MessageT>
void IntraDispatcher::AddListener(const RoleAttributes& self_attr,
                                  const MessageListener<MessageT>& listener) {
  if (is_shutdown_.load()) {
    return;
  }

  auto channel_id = self_attr.channel_id();
  std::string message_type = message::GetMessageName<MessageT>();
  uint64_t self_id = self_attr.id();

  bool created =
      chain_->AddListener(self_id, channel_id, message_type, listener);

  auto handler = GetHandler<MessageT>(self_attr.channel_id());
  if (handler && created) {
    auto listener_wrapper =
                      [this, self_id, channel_id, message_type, self_attr](
                                const std::shared_ptr<MessageT>& message,
                                const MessageInfo& message_info) {
      auto recv_time = Time::Now().ToMicrosecond();
      statistics::Statistics::Instance()->SetProcStatus(self_attr, recv_time);
      this->chain_->Run<MessageT>(self_id, channel_id, message_type, message,
                                  message_info);
    };
    handler->Connect(self_id, listener_wrapper);
  }
}

template <typename MessageT>
void IntraDispatcher::AddListener(const RoleAttributes& self_attr,
                                  const RoleAttributes& opposite_attr,
                                  const MessageListener<MessageT>& listener) {
  if (is_shutdown_.load()) {
    return;
  }

  auto channel_id = self_attr.channel_id();
  std::string message_type = message::GetMessageName<MessageT>();
  uint64_t self_id = self_attr.id();
  uint64_t oppo_id = opposite_attr.id();

  bool created =
      chain_->AddListener(self_id, oppo_id, channel_id, message_type, listener);

  auto handler = GetHandler<MessageT>(self_attr.channel_id());
  if (handler && created) {
    auto listener_wrapper =
              [this, self_id, oppo_id, channel_id, message_type, self_attr](
                                const std::shared_ptr<MessageT>& message,
                                const MessageInfo& message_info) {
      auto recv_time = Time::Now().ToMicrosecond();
      statistics::Statistics::Instance()->SetProcStatus(self_attr, recv_time);
      this->chain_->Run<MessageT>(self_id, oppo_id, channel_id, message_type,
                                  message, message_info);
    };
    handler->Connect(self_id, oppo_id, listener_wrapper);
  }
}

template <typename MessageT>
void IntraDispatcher::RemoveListener(const RoleAttributes& self_attr) {
  if (is_shutdown_.load()) {
    return;
  }
  Dispatcher::RemoveListener<MessageT>(self_attr);
  chain_->RemoveListener<MessageT>(self_attr.id(), self_attr.channel_id(),
                                   message::GetMessageName<MessageT>());
}

template <typename MessageT>
void IntraDispatcher::RemoveListener(const RoleAttributes& self_attr,
                                     const RoleAttributes& opposite_attr) {
  if (is_shutdown_.load()) {
    return;
  }
  Dispatcher::RemoveListener<MessageT>(self_attr, opposite_attr);
  chain_->RemoveListener<MessageT>(self_attr.id(), opposite_attr.id(),
                                   self_attr.channel_id(),
                                   message::GetMessageName<MessageT>());
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_DISPATCHER_INTRA_DISPATCHER_H_
