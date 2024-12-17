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

#include "cyber/common/log.h"
#include "cyber/statistics/statistics.h"
#include "cyber/transport/common/endpoint.h"
#include "cyber/transport/dispatcher/dispatcher.h"

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
      item.second = nullptr;
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
  // subscriber exsit
  if (subs_.count(channel_id) > 0) {
    return;
  }

  auto listener_adapter =
      [this, self_attr](const std::shared_ptr<std::string>& msg_str,
                        uint64_t channel_id, const MessageInfo& msg_info) {
        statistics::Statistics::Instance()->AddRecvCount(self_attr,
                                                         msg_info.seq_num());
        statistics::Statistics::Instance()->SetTotalMsgsStatus(
            self_attr, msg_info.seq_num());
        this->OnMessage(channel_id, msg_str, msg_info);
      };

  auto& qos = self_attr.qos_profile();
  auto subscriber_ptr = participant_->CreateSubscriber(self_attr.channel_name(),
                                                       qos, listener_adapter);
  RETURN_IF_NULL(subscriber_ptr);

  subs_[channel_id] = subscriber_ptr;
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
