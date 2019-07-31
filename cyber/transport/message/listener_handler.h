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

#ifndef CYBER_TRANSPORT_MESSAGE_LISTENER_HANDLER_H_
#define CYBER_TRANSPORT_MESSAGE_LISTENER_HANDLER_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/base/signal.h"
#include "cyber/common/log.h"
#include "cyber/message/raw_message.h"
#include "cyber/transport/message/message_info.h"

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace apollo {
namespace cyber {
namespace transport {

using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;

class ListenerHandlerBase;
using ListenerHandlerBasePtr = std::shared_ptr<ListenerHandlerBase>;

class ListenerHandlerBase {
 public:
  ListenerHandlerBase() {}
  virtual ~ListenerHandlerBase() {}

  virtual void Disconnect(uint64_t self_id) = 0;
  virtual void Disconnect(uint64_t self_id, uint64_t oppo_id) = 0;
  inline bool IsRawMessage() const { return is_raw_message_; }
  virtual void RunRaw(const std::shared_ptr<message::RawMessage>& message,
                      const MessageInfo& msg_info) = 0;

 protected:
  bool is_raw_message_ = false;
};

template <typename MessageT>
class ListenerHandlerCore : public ListenerHandlerBase {
 public:
  using Message = std::shared_ptr<MessageT>;
  using MessageSignal = base::Signal<const Message&, const MessageInfo&>;

  using Listener = std::function<void(const Message&, const MessageInfo&)>;
  using MessageConnection =
      base::Connection<const Message&, const MessageInfo&>;
  using ConnectionMap = std::unordered_map<uint64_t, MessageConnection>;

  ListenerHandlerCore() {}
  virtual ~ListenerHandlerCore() {}

  void Connect(uint64_t self_id, const Listener& listener);
  void Connect(uint64_t self_id, uint64_t oppo_id, const Listener& listener);

  void Disconnect(uint64_t self_id) override;
  void Disconnect(uint64_t self_id, uint64_t oppo_id) override;

  void Run(const Message& msg, const MessageInfo& msg_info);

 private:
  using SignalPtr = std::shared_ptr<MessageSignal>;
  using MessageSignalMap = std::unordered_map<uint64_t, SignalPtr>;
  // used for self_id
  MessageSignal signal_;
  ConnectionMap signal_conns_;  // key: self_id

  // used for self_id and oppo_id
  MessageSignalMap signals_;  // key: oppo_id
  // key: oppo_id
  std::unordered_map<uint64_t, ConnectionMap> signals_conns_;

  base::AtomicRWLock rw_lock_;
};

template <typename MessageT,
          bool = std::is_base_of<google::protobuf::Message, MessageT>::value>
class ListenerHandler : public ListenerHandlerCore<MessageT> {
  inline void RunRaw(const std::shared_ptr<message::RawMessage>& message,
                     const MessageInfo& msg_info) override {
    AFATAL << typeid(MessageT).name() << " not supported.";
  }
};

template <typename MessageT>
class ListenerHandler<MessageT, true> : public ListenerHandlerCore<MessageT> {
  inline void RunRaw(const std::shared_ptr<message::RawMessage>& message,
                     const MessageInfo& msg_info) override {
    auto protobuf_msg = std::make_shared<MessageT>();
    CHECK(protobuf_msg->ParseFromString(message->message))
        << "failed to parse \"" << message->message << "\" into "
        << typeid(MessageT).name();
    this->Run(protobuf_msg, msg_info);
  }
};

template <>
class ListenerHandler<message::RawMessage, false>
    : public ListenerHandlerCore<message::RawMessage> {
  inline void RunRaw(const std::shared_ptr<message::RawMessage>& message,
                     const MessageInfo& msg_info) override {
    this->Run(message, msg_info);
  }
};

template <>
inline ListenerHandlerCore<message::RawMessage>::ListenerHandlerCore() {
  is_raw_message_ = true;
}

template <typename MessageT>
void ListenerHandlerCore<MessageT>::Connect(uint64_t self_id,
                                            const Listener& listener) {
  auto connection = signal_.Connect(listener);
  if (!connection.IsConnected()) {
    return;
  }

  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  signal_conns_[self_id] = connection;
}

template <typename MessageT>
void ListenerHandlerCore<MessageT>::Connect(uint64_t self_id, uint64_t oppo_id,
                                            const Listener& listener) {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  if (signals_.find(oppo_id) == signals_.end()) {
    signals_[oppo_id] = std::make_shared<MessageSignal>();
  }

  auto connection = signals_[oppo_id]->Connect(listener);
  if (!connection.IsConnected()) {
    AWARN << oppo_id << " " << self_id << "connect failed!";
    return;
  }

  if (signals_conns_.find(oppo_id) == signals_conns_.end()) {
    signals_conns_[oppo_id] = ConnectionMap();
  }

  signals_conns_[oppo_id][self_id] = connection;
}

template <typename MessageT>
void ListenerHandlerCore<MessageT>::Disconnect(uint64_t self_id) {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  if (signal_conns_.find(self_id) == signal_conns_.end()) {
    return;
  }

  signal_conns_[self_id].Disconnect();
  signal_conns_.erase(self_id);
}

template <typename MessageT>
void ListenerHandlerCore<MessageT>::Disconnect(uint64_t self_id,
                                               uint64_t oppo_id) {
  WriteLockGuard<AtomicRWLock> lock(rw_lock_);
  if (signals_conns_.find(oppo_id) == signals_conns_.end()) {
    return;
  }

  if (signals_conns_[oppo_id].find(self_id) == signals_conns_[oppo_id].end()) {
    return;
  }

  signals_conns_[oppo_id][self_id].Disconnect();
  signals_conns_[oppo_id].erase(self_id);
}

template <typename MessageT>
void ListenerHandlerCore<MessageT>::Run(const Message& msg,
                                        const MessageInfo& msg_info) {
  signal_(msg, msg_info);
  uint64_t oppo_id = msg_info.sender_id().HashValue();
  ReadLockGuard<AtomicRWLock> lock(rw_lock_);
  if (signals_.find(oppo_id) == signals_.end()) {
    return;
  }

  (*signals_[oppo_id])(msg, msg_info);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_MESSAGE_LISTENER_HANDLER_H_
