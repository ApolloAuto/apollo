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

#ifndef CYBERTRON_TRANSPORT_DISPATCHER_DISPATCHER_H_
#define CYBERTRON_TRANSPORT_DISPATCHER_DISPATCHER_H_

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "cybertron/base/atomic_hash_map.h"
#include "cybertron/base/atomic_rw_lock.h"
#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/proto/role_attributes.pb.h"
#include "cybertron/transport/message/listener_handler.h"
#include "cybertron/transport/message/message_info.h"

namespace apollo {
namespace cybertron {
namespace transport {

using apollo::cybertron::base::AtomicHashMap;
using apollo::cybertron::base::AtomicRWLock;
using apollo::cybertron::base::ReadLockGuard;
using apollo::cybertron::base::WriteLockGuard;
using apollo::cybertron::common::GlobalData;
using cybertron::proto::RoleAttributes;

class Dispatcher;
using DispatcherPtr = std::shared_ptr<Dispatcher>;

template <typename MessageT>
using MessageListener =
    std::function<void(const std::shared_ptr<MessageT>&, const MessageInfo&)>;

class Dispatcher {
 public:
  Dispatcher();
  virtual ~Dispatcher();

  virtual void Shutdown();

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

  bool HasChannel(uint64_t channel_id);

 protected:
  bool shutdown_;
  // key: channel_id of message
  AtomicHashMap<uint64_t, ListenerHandlerBasePtr> msg_listeners_;
  base::AtomicRWLock rw_lock_;
};

template <typename MessageT>
void Dispatcher::AddListener(const RoleAttributes& self_attr,
                             const MessageListener<MessageT>& listener) {
  uint64_t channel_id = self_attr.channel_id();

  std::shared_ptr<ListenerHandler<MessageT>> handler;
  ListenerHandlerBasePtr* handler_base = nullptr;
  if (msg_listeners_.Get(channel_id, &handler_base)) {
    handler =
        std::dynamic_pointer_cast<ListenerHandler<MessageT>>(*handler_base);
  } else {
    ADEBUG << "new reader for channel:"
           << GlobalData::GetChannelById(channel_id);
    handler.reset(new ListenerHandler<MessageT>());
    msg_listeners_.Set(channel_id, handler);
  }
  handler->Connect(self_attr.id(), listener);
}

template <typename MessageT>
void Dispatcher::AddListener(const RoleAttributes& self_attr,
                             const RoleAttributes& opposite_attr,
                             const MessageListener<MessageT>& listener) {
  uint64_t channel_id = self_attr.channel_id();

  std::shared_ptr<ListenerHandler<MessageT>> handler;
  ListenerHandlerBasePtr* handler_base = nullptr;
  if (msg_listeners_.Get(channel_id, &handler_base)) {
    handler =
        std::dynamic_pointer_cast<ListenerHandler<MessageT>>(*handler_base);
  } else {
    ADEBUG << "new reader for channel:"
           << GlobalData::GetChannelById(channel_id);
    handler.reset(new ListenerHandler<MessageT>());
    msg_listeners_.Set(channel_id, handler);
  }
  handler->Connect(self_attr.id(), opposite_attr.id(), listener);
}

template <typename MessageT>
void Dispatcher::RemoveListener(const RoleAttributes& self_attr) {
  uint64_t channel_id = self_attr.channel_id();

  ListenerHandlerBasePtr* handler_base = nullptr;
  if (msg_listeners_.Get(channel_id, &handler_base)) {
    (*handler_base)->Disconnect(self_attr.id());
  }
}

template <typename MessageT>
void Dispatcher::RemoveListener(const RoleAttributes& self_attr,
                                const RoleAttributes& opposite_attr) {
  uint64_t channel_id = self_attr.channel_id();

  ListenerHandlerBasePtr* handler_base = nullptr;
  if (msg_listeners_.Get(channel_id, &handler_base)) {
    (*handler_base)->Disconnect(self_attr.id(), opposite_attr.id());
  }
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_DISPATCHER_DISPATCHER_H_
