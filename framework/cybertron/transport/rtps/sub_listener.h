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

#ifndef CYBERTRON_TRANSPORT_RTPS_SUB_LISTENER_H_
#define CYBERTRON_TRANSPORT_RTPS_SUB_LISTENER_H_

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <fastrtps/Domain.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/subscriber/SubscriberListener.h>

#include "cybertron/transport/message/message_info.h"
#include "cybertron/transport/rtps/underlay_message.h"
#include "cybertron/transport/rtps/underlay_message_type.h"

namespace apollo {
namespace cybertron {
namespace transport {

class SubListener;
using SubListenerPtr = std::shared_ptr<SubListener>;

class SubListener : public eprosima::fastrtps::SubscriberListener {
 public:
  using NewMsgCallback = std::function<void(
      uint64_t channel_id, const std::shared_ptr<std::string>& msg_str,
      const MessageInfo& msg_info)>;

  explicit SubListener(const NewMsgCallback& callback);
  virtual ~SubListener();

  void onNewDataMessage(eprosima::fastrtps::Subscriber* sub);
  void onSubscriptionMatched(eprosima::fastrtps::Subscriber* sub,
                             eprosima::fastrtps::MatchingInfo& info);

 private:
  NewMsgCallback callback_;
  MessageInfo msg_info_;
  std::mutex mutex_;
};

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_RTPS_SUB_LISTENER_H_
