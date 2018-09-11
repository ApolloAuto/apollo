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

#ifndef CYBERTRON_TRANSPORT_UPPER_REACH_INTRA_UPPER_REACH_H_
#define CYBERTRON_TRANSPORT_UPPER_REACH_INTRA_UPPER_REACH_H_

#include <memory>
#include <string>

#include "cybertron/common/log.h"
#include "cybertron/transport/dispatcher/intra_dispatcher.h"
#include "cybertron/transport/upper_reach/upper_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

template <typename MessageT>
class IntraUpperReach : public UpperReach<MessageT> {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;

  explicit IntraUpperReach(const RoleAttributes& attr);
  virtual ~IntraUpperReach();

  void Enable() override;
  void Disable() override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

 private:
  uint64_t channel_id_;
  IntraDispatcherPtr dispatcher_;
};

template <typename MessageT>
IntraUpperReach<MessageT>::IntraUpperReach(const RoleAttributes& attr)
    : UpperReach<MessageT>(attr),
      channel_id_(attr.channel_id()),
      dispatcher_(nullptr) {}

template <typename MessageT>
IntraUpperReach<MessageT>::~IntraUpperReach() {
  Disable();
}

template <typename MessageT>
void IntraUpperReach<MessageT>::Enable() {
  if (!this->enabled_) {
    dispatcher_ = IntraDispatcher::Instance();
    this->enabled_ = true;
  }
}

template <typename MessageT>
void IntraUpperReach<MessageT>::Disable() {
  if (this->enabled_) {
    dispatcher_ = nullptr;
    this->enabled_ = false;
  }
}

template <typename MessageT>
bool IntraUpperReach<MessageT>::Transmit(const MessagePtr& msg,
                                         const MessageInfo& msg_info) {
  if (!this->enabled_) {
    ADEBUG << "not enable.";
    return false;
  }
  dispatcher_->OnMessage(channel_id_, msg, msg_info);
  return true;
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_UPPER_REACH_INTRA_UPPER_REACH_H_
