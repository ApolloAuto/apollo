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

#ifndef CYBERTRON_TRANSPORT_UPPER_REACH_UPPER_REACH_H_
#define CYBERTRON_TRANSPORT_UPPER_REACH_UPPER_REACH_H_

#include <cstdint>
#include <memory>
#include <string>

#include "cybertron/event/perf_event_cache.h"
#include "cybertron/transport/common/endpoint.h"
#include "cybertron/transport/message/message_info.h"

namespace apollo {
namespace cybertron {
namespace transport {

using apollo::cybertron::event::PerfEventCache;
using apollo::cybertron::event::TransPerf;

template <typename MessageT>
class UpperReach : public Endpoint {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;

  explicit UpperReach(const RoleAttributes& attr);
  virtual ~UpperReach();

  virtual void Enable() = 0;
  virtual void Disable() = 0;

  virtual void Enable(const RoleAttributes& opposite_attr);
  virtual void Disable(const RoleAttributes& opposite_attr);

  virtual bool Transmit(const MessagePtr& msg);
  virtual bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) = 0;

  uint64_t NextSeqNum() { return ++seq_num_; }

  uint64_t seq_num() const { return seq_num_; }

 protected:
  uint64_t seq_num_;
  MessageInfo msg_info_;
};

template <typename MessageT>
UpperReach<MessageT>::UpperReach(const RoleAttributes& attr)
    : Endpoint(attr), seq_num_(0) {
  msg_info_.set_sender_id(this->id_);
  msg_info_.set_seq_num(this->seq_num_);
}

template <typename MessageT>
UpperReach<MessageT>::~UpperReach() {}

template <typename MessageT>
bool UpperReach<MessageT>::Transmit(const MessagePtr& msg) {
  msg_info_.set_seq_num(NextSeqNum());
  PerfEventCache::Instance()->AddTransportEvent(
      TransPerf::TRANS_FROM, attr_.channel_id(), msg_info_.seq_num());
  return Transmit(msg, msg_info_);
}

template <typename MessageT>
void UpperReach<MessageT>::Enable(const RoleAttributes& opposite_attr) {
  (void)opposite_attr;
  Enable();
}

template <typename MessageT>
void UpperReach<MessageT>::Disable(const RoleAttributes& opposite_attr) {
  (void)opposite_attr;
  Disable();
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_UPPER_REACH_UPPER_REACH_H_
