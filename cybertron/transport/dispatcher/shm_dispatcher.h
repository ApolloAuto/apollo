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

#ifndef CYBERTRON_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_
#define CYBERTRON_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "cybertron/base/atomic_rw_lock.h"
#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "cybertron/message/message_traits.h"
#include "cybertron/proto/transport_conf.pb.h"
#include "cybertron/transport/common/syscall_wrapper.h"
#include "cybertron/transport/dispatcher/dispatcher.h"
#include "cybertron/transport/shm/segment.h"

namespace apollo {
namespace cybertron {
namespace transport {

class ShmDispatcher;
using ShmDispatcherPtr = std::shared_ptr<ShmDispatcher>;
using apollo::cybertron::base::AtomicRWLock;
using apollo::cybertron::base::ReadLockGuard;
using apollo::cybertron::base::WriteLockGuard;

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

 private:
  void AddSegment(const RoleAttributes& self_attr);
  void OnMessage(uint64_t channel_id,
                 const std::shared_ptr<std::string>& msg_str,
                 const MessageInfo& msg_info);
  void ThreadFunc();
  bool Init();
  bool InitMulticast();

  uint64_t host_id_;
  SegmentContainer segments_;
  AtomicRWLock segments_lock_;
  std::thread thread_;
  int sfd_;
  struct sockaddr_in local_addr_;
  std::shared_ptr<proto::ShmMulticastLocator> locator_;

  DECLARE_SINGLETON(ShmDispatcher)
};

// template <>
// void ShmDispatcher::AddListener(
//    const RoleAttributes& self_attr,
//    const std::function<void(const std::shared_ptr<message::RawMessage>&,
//                             const MessageInfo&)>& listener);

template <typename MessageT>
void ShmDispatcher::AddListener(const RoleAttributes& self_attr,
                                const MessageListener<MessageT>& listener) {
  auto listener_adapter = [listener](
      const std::shared_ptr<std::string>& msg_str,
      const MessageInfo& msg_info) {
    auto msg = std::make_shared<MessageT>();
    RETURN_IF(!message::ParseFromString(*msg_str, msg.get()));
    listener(msg, msg_info);
  };

  Dispatcher::AddListener<std::string>(self_attr, listener_adapter);
  AddSegment(self_attr);
}

// template <>
// void ShmDispatcher::AddListener(
//    const RoleAttributes& self_attr, const RoleAttributes& opposite_attr,
//    const std::function<void(const std::shared_ptr<message::RawMessage>&,
//                             const MessageInfo&)>& listener);

template <typename MessageT>
void ShmDispatcher::AddListener(const RoleAttributes& self_attr,
                                const RoleAttributes& opposite_attr,
                                const MessageListener<MessageT>& listener) {
  auto listener_adapter = [listener](
      const std::shared_ptr<std::string>& msg_str,
      const MessageInfo& msg_info) {
    auto msg = std::make_shared<MessageT>();
    RETURN_IF(!message::ParseFromString(*msg_str, msg.get()));
    listener(msg, msg_info);
  };

  Dispatcher::AddListener<std::string>(self_attr, opposite_attr,
                                       listener_adapter);
  AddSegment(self_attr);
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_DISPATCHER_SHM_DISPATCHER_H_
