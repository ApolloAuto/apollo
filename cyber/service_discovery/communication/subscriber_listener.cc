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

#include "cyber/service_discovery/communication/subscriber_listener.h"

#include <memory>

#include "cyber/base/macros.h"

#include "fastdds/dds/subscriber/InstanceState.hpp"

#include "fastdds/dds/topic/TopicDescription.hpp"

#include "cyber/common/log.h"
#include "cyber/transport/rtps/underlay_message.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

SubscriberListener::SubscriberListener(
    const transport::rtps::subsciber_callback& callback)
    : callback_(callback) {}

SubscriberListener::~SubscriberListener() {
  std::lock_guard<std::mutex> lck(mutex_);
  callback_ = nullptr;
}

void SubscriberListener::on_data_available(
    eprosima::fastdds::dds::DataReader* reader) {
  RETURN_IF_NULL(callback_);

  eprosima::fastdds::dds::SampleInfo m_info;
  cyber::transport::UnderlayMessage m;
  while (reader->take_next_sample(reinterpret_cast<void*>(&m), &m_info) ==
         eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
    if (m_info.valid_data) {
      // parameter 1 and 2 are not used
      callback_(std::make_shared<std::string>(m.data()), 0, msg_info_);
    } else {
      AERROR << "Remote writer for topic "
             << reader->get_topicdescription()->get_name() << " is dead";
    }
  }
}
//这是apollo-fei-local-1的提交
void SubscriberListener::on_subscription_matched(
    eprosima::fastdds::dds::DataReader* reader,
    const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) {
  (void)reader;
  (void)info;
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
