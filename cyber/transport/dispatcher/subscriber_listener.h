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

#ifndef CYBER_TRANSPORT_DISPATCHER_SUBSCRIBER_LISTENER_H_
#define CYBER_TRANSPORT_DISPATCHER_SUBSCRIBER_LISTENER_H_

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "cyber/base/macros.h"

#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/SampleInfo.hpp"
#include "fastdds/dds/subscriber/SubscriberListener.hpp"
#include "fastdds/dds/topic/TopicDescription.hpp"

#include "cyber/common/util.h"
#include "cyber/transport/message/message_info.h"
#include "cyber/transport/common/common_type.h"
#include "cyber/transport/rtps/underlay_message.h"
#include "cyber/transport/rtps/underlay_message_type.h"

namespace apollo {
namespace cyber {
namespace transport {
namespace dispatcher {
class SubscriberListener : public eprosima::fastdds::dds::SubscriberListener {
 public:
  explicit SubscriberListener(const rtps::subsciber_callback& callback);
  virtual ~SubscriberListener();

  void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;
  void on_subscription_matched(
      eprosima::fastdds::dds::DataReader* reader,
      const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
      override;  // NOLINT

 private:
  rtps::subsciber_callback callback_;
  MessageInfo msg_info_;
  std::mutex mutex_;
};

}  // namespace dispatcher
}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_DISPATCHER_SUBSCRIBER_LISTENER_H_
