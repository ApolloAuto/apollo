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

#include "cybertron/topology/communication/subscriber_listener.h"

#include "cybertron/common/log.h"
#include "cybertron/transport/rtps/underlay_message.h"
#include "cybertron/transport/rtps/underlay_message_type.h"

namespace apollo {
namespace cybertron {
namespace topology {

SubscriberListener::SubscriberListener(const NewMsgCallback& callback)
    : callback_(callback) {}

SubscriberListener::~SubscriberListener() {
  std::lock_guard<std::mutex> lck(mutex_);
  callback_ = nullptr;
}

void SubscriberListener::onNewDataMessage(eprosima::fastrtps::Subscriber* sub) {
  RETURN_IF_NULL(callback_);

  std::lock_guard<std::mutex> lock(mutex_);
  eprosima::fastrtps::SampleInfo_t m_info;
  cybertron::transport::UnderlayMessage m;
  RETURN_IF(!sub->takeNextData(reinterpret_cast<void*>(&m), &m_info));
  RETURN_IF(m_info.sampleKind != eprosima::fastrtps::ALIVE);

  callback_(m.data());
}

void SubscriberListener::onSubscriptionMatched(
    eprosima::fastrtps::Subscriber* sub,
    eprosima::fastrtps::MatchingInfo& info) {
  (void)sub;
  (void)info;
}

}  // namespace topology
}  // namespace cybertron
}  // namespace apollo
