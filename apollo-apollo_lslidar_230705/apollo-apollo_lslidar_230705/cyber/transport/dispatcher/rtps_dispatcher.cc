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

#include "cyber/transport/dispatcher/rtps_dispatcher.h"

namespace apollo {
namespace cyber {
namespace transport {

RtpsDispatcher::RtpsDispatcher() : participant_(nullptr) {}

RtpsDispatcher::~RtpsDispatcher() { Shutdown(); }

void RtpsDispatcher::Shutdown() {
  if (is_shutdown_.exchange(true)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(subs_mutex_);
    for (auto& item : subs_) {
      item.second.sub = nullptr;
    }
  }

  participant_ = nullptr;
}

void RtpsDispatcher::AddSubscriber(const RoleAttributes& self_attr) {
  if (participant_ == nullptr) {
    AWARN << "please set participant firstly.";
    return;
  }

  uint64_t channel_id = self_attr.channel_id();
  std::lock_guard<std::mutex> lock(subs_mutex_);
  if (subs_.count(channel_id) > 0) {
    return;
  }

  Subscriber new_sub;
  eprosima::fastrtps::SubscriberAttributes sub_attr;
  auto& qos = self_attr.qos_profile();
  RETURN_IF(!AttributesFiller::FillInSubAttr(self_attr.channel_name(), qos,
                                             &sub_attr));

  new_sub.sub_listener = std::make_shared<SubListener>(
      std::bind(&RtpsDispatcher::OnMessage, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  new_sub.sub = eprosima::fastrtps::Domain::createSubscriber(
      participant_->fastrtps_participant(), sub_attr,
      new_sub.sub_listener.get());
  RETURN_IF_NULL(new_sub.sub);
  subs_[channel_id] = new_sub;
}

void RtpsDispatcher::OnMessage(uint64_t channel_id,
                               const std::shared_ptr<std::string>& msg_str,
                               const MessageInfo& msg_info) {
  if (is_shutdown_.load()) {
    return;
  }

  ListenerHandlerBasePtr* handler_base = nullptr;
  if (msg_listeners_.Get(channel_id, &handler_base)) {
    auto handler =
        std::dynamic_pointer_cast<ListenerHandler<std::string>>(*handler_base);
    handler->Run(msg_str, msg_info);
  }
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
