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

#ifndef CYBERTRON_TRANSPORT_LOWER_REACH_LOWER_REACH_H_
#define CYBERTRON_TRANSPORT_LOWER_REACH_LOWER_REACH_H_

#include <functional>
#include <memory>

#include "cybertron/transport/common/endpoint.h"
#include "cybertron/transport/message/history.h"
#include "cybertron/transport/message/message_info.h"

namespace apollo {
namespace cybertron {
namespace transport {

template <typename MessageT>
class LowerReach : public Endpoint {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;
  using MessageListener = std::function<void(
      const MessagePtr&, const MessageInfo&, const RoleAttributes&)>;

  LowerReach(const RoleAttributes& attr, const MessageListener& msg_listener);
  virtual ~LowerReach();

  virtual void Enable() = 0;
  virtual void Disable() = 0;
  virtual void Enable(const RoleAttributes& opposite_attr) = 0;
  virtual void Disable(const RoleAttributes& opposite_attr) = 0;

 protected:
  void OnNewMessage(const MessagePtr& msg, const MessageInfo& msg_info);

  MessageListener msg_listener_;
};

template <typename MessageT>
LowerReach<MessageT>::LowerReach(const RoleAttributes& attr,
                                 const MessageListener& msg_listener)
    : Endpoint(attr), msg_listener_(msg_listener) {}

template <typename MessageT>
LowerReach<MessageT>::~LowerReach() {}

template <typename MessageT>
void LowerReach<MessageT>::OnNewMessage(const MessagePtr& msg,
                                        const MessageInfo& msg_info) {
  if (msg_listener_ != nullptr) {
    msg_listener_(msg, msg_info, attr_);
  }
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_LOWER_REACH_LOWER_REACH_H_
