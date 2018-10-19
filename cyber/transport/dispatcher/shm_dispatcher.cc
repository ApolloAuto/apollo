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

#include "cyber/transport/dispatcher/shm_dispatcher.h"
#include "cyber/common/global_data.h"
#include "cyber/common/util.h"
#include "cyber/transport/shm/readable_info.h"

using apollo::cyber::common::GlobalData;

namespace apollo {
namespace cyber {
namespace transport {

ShmDispatcher::ShmDispatcher() : host_id_(0), sfd_(-1) { Init(); }

ShmDispatcher::~ShmDispatcher() { Shutdown(); }

void ShmDispatcher::Shutdown() {
  if (shutdown_) {
    return;
  }
  shutdown_ = true;
  segments_.clear();
  if (thread_.joinable()) {
    thread_.join();
  }
  CloseAndReset(&sfd_);
}

void ShmDispatcher::AddSegment(const RoleAttributes& self_attr) {
  uint64_t channel_id = self_attr.channel_id();
  WriteLockGuard<AtomicRWLock> lock(segments_lock_);
  if (segments_.count(channel_id) > 0) {
    return;
  }
  auto segment = std::make_shared<Segment>(channel_id, READ_ONLY);
  segments_[channel_id] = segment;
}

void ShmDispatcher::OnMessage(uint64_t channel_id,
                              const std::shared_ptr<std::string>& msg_str,
                              const MessageInfo& msg_info) {
  ListenerHandlerBasePtr* handler_base = nullptr;
  if (msg_listeners_.Get(channel_id, &handler_base)) {
    auto handler =
        std::dynamic_pointer_cast<ListenerHandler<std::string>>(*handler_base);
    handler->Run(msg_str, msg_info);
  }
}

void ShmDispatcher::ThreadFunc() {
  char buf[1024] = {0};
  int addr_len = sizeof(local_addr_);
  MessageInfo msg_info;
  std::string msg_info_str("");
  std::shared_ptr<std::string> msg_str_ptr = std::make_shared<std::string>();

  while (!shutdown_) {
    int read_bytes =
        recvfrom(sfd_, buf, 1024, 0, (struct sockaddr*)&local_addr_,
                 reinterpret_cast<socklen_t*>(&addr_len));
    if (read_bytes == -1) {
      usleep(10);
      continue;
    }

    std::string readable_info_str(buf, read_bytes);
    ReadableInfo readable_info;
    readable_info.DeserializeFrom(readable_info_str);
    uint64_t host_id = readable_info.host_id();
    if (host_id != host_id_) {
      ADEBUG << "shm readable info from other host.";
      continue;
    }

    uint64_t channel_id = readable_info.channel_id();
    uint32_t block_index = readable_info.block_index();

    {
      ReadLockGuard<AtomicRWLock> lock(segments_lock_);
      if (segments_.count(channel_id) > 0) {
        if (!segments_[channel_id]->Read(block_index, msg_str_ptr.get(),
                                         &msg_info_str)) {
          AERROR << "read msg failed, channel:"
                 << GlobalData::GetChannelById(channel_id);
          continue;
        }
        if (msg_info.DeserializeFrom(msg_info_str)) {
          OnMessage(channel_id, msg_str_ptr, msg_info);
        } else {
          AERROR << "error msg info of channel:"
                 << GlobalData::GetChannelById(channel_id);
        }
      }
    }
  }
}

bool ShmDispatcher::Init() {
  host_id_ = common::Hash(common::GlobalData::Instance()->HostIp());

  if (!InitMulticast()) {
    CloseAndReset(&sfd_);
    return false;
  }

  thread_ = std::thread(&ShmDispatcher::ThreadFunc, this);
  return true;
}

bool ShmDispatcher::InitMulticast() {
  auto global_conf = common::GlobalData::Instance()->Config();
  RETURN_VAL_IF(!global_conf.has_transport_conf(), false);
  RETURN_VAL_IF(!global_conf.transport_conf().has_shm_locator(), false);

  locator_ = std::make_shared<proto::ShmMulticastLocator>();
  locator_->CopyFrom(global_conf.transport_conf().shm_locator());

  sfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  RETURN_VAL_IF(sfd_ == -1, false);
  RETURN_VAL_IF(!SetNonBlocking(sfd_), false);

  memset(&local_addr_, 0, sizeof(local_addr_));
  local_addr_.sin_family = AF_INET;
  local_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr_.sin_port = htons(locator_->port());

  int yes = 1;
  RETURN_VAL_IF(
      setsockopt(sfd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0, false);

  RETURN_VAL_IF(
      bind(sfd_, (struct sockaddr*)&local_addr_, sizeof(local_addr_)) < 0,
      false);

  int loop = 1;
  RETURN_VAL_IF(
      setsockopt(sfd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0,
      false);

  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(locator_->ip().c_str());
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  RETURN_VAL_IF(
      setsockopt(sfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0,
      false);

  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
