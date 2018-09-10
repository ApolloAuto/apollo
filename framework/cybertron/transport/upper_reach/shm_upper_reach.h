/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBERTRON_TRANSPORT_UPPER_REACH_SHM_UPPER_REACH_H_
#define CYBERTRON_TRANSPORT_UPPER_REACH_SHM_UPPER_REACH_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/util.h"
#include "cybertron/message/message_traits.h"
#include "cybertron/proto/transport_conf.pb.h"
#include "cybertron/transport/common/syscall_wrapper.h"
#include "cybertron/transport/shm/readable_info.h"
#include "cybertron/transport/shm/segment.h"
#include "cybertron/transport/upper_reach/upper_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

template <typename MessageT>
class ShmUpperReach : public UpperReach<MessageT> {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;

  explicit ShmUpperReach(const RoleAttributes& attr);
  virtual ~ShmUpperReach();

  void Enable() override;
  void Disable() override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

 private:
  bool Transmit(const MessageT& msg, const MessageInfo& msg_info);

  SegmentPtr segment_;
  std::string channel_name_;
  uint64_t channel_id_;
  uint64_t host_id_;
  int sfd_;
  struct sockaddr_in mcast_addr_;
  std::shared_ptr<proto::ShmMulticastLocator> locator_;
};

template <typename MessageT>
ShmUpperReach<MessageT>::ShmUpperReach(const RoleAttributes& attr)
    : UpperReach<MessageT>(attr),
      segment_(nullptr),
      channel_name_(attr.channel_name()),
      channel_id_(attr.channel_id()),
      sfd_(-1),
      locator_(nullptr) {
  host_id_ = common::Hash(attr.host_name());
  memset(&mcast_addr_, 0, sizeof(mcast_addr_));
  locator_ = std::make_shared<proto::ShmMulticastLocator>();
}

template <typename MessageT>
ShmUpperReach<MessageT>::~ShmUpperReach() {
  Disable();
}

template <typename MessageT>
void ShmUpperReach<MessageT>::Enable() {
  if (this->enabled_) {
    return;
  }

  auto global_conf = common::GlobalData::Instance()->Config();
  RETURN_IF(!global_conf.has_transport_conf());
  RETURN_IF(!global_conf.transport_conf().has_shm_locator());
  locator_->CopyFrom(global_conf.transport_conf().shm_locator());

  sfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  RETURN_IF(sfd_ == -1);

  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(locator_->ip().c_str());
  mcast_addr_.sin_port = htons(locator_->port());

  segment_ = std::make_shared<Segment>(channel_id_, WRITE_ONLY);
  this->enabled_ = true;
}

template <typename MessageT>
void ShmUpperReach<MessageT>::Disable() {
  if (this->enabled_) {
    segment_ = nullptr;
    CloseAndReset(&sfd_);
    this->enabled_ = false;
  }
}

template <typename MessageT>
bool ShmUpperReach<MessageT>::Transmit(const MessagePtr& msg,
                                       const MessageInfo& msg_info) {
  return Transmit(*msg, msg_info);
}

template <typename MessageT>
bool ShmUpperReach<MessageT>::Transmit(const MessageT& msg,
                                       const MessageInfo& msg_info) {
  if (!this->enabled_) {
    ADEBUG << "not enable.";
    return false;
  }
  std::string msg_str = "";
  RETURN_VAL_IF(!message::SerializeToString(msg, &msg_str), false);

  std::string msg_info_str = "";
  msg_info.SerializeTo(&msg_info_str);
  uint32_t block_index = 0;

  RETURN_VAL_IF(!segment_->Write(msg_str, msg_info_str, &block_index), false);

  ReadableInfo readable_info(host_id_, block_index, channel_id_);
  std::string readable_info_str = "";
  readable_info.SerializeTo(&readable_info_str);
  int write_bytes =
      sendto(sfd_, readable_info_str.c_str(), readable_info_str.size(), 0,
             (struct sockaddr*)&mcast_addr_, sizeof(mcast_addr_));
  RETURN_VAL_IF(write_bytes < 0, false);
  return true;
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_UPPER_REACH_SHM_UPPER_REACH_H_
