/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "cyber/transport/dispatcher/subscriber_listener.h"

#include "cyber/common/log.h"
#include "cyber/transport/message/message_info.h"

namespace apollo {
namespace cyber {
namespace transport {
namespace dispatcher {

SubscriberListener::SubscriberListener(const rtps::subsciber_callback& callback)
    : callback_(callback) {}

SubscriberListener::~SubscriberListener() {}

void SubscriberListener::on_data_available(
    eprosima::fastdds::dds::DataReader* reader) {
  RETURN_IF_NULL(reader);
  RETURN_IF_NULL(callback_);

  // fetch channel name
  auto channel_id = common::Hash(reader->get_topicdescription()->get_name());
  eprosima::fastdds::dds::SampleInfo m_info;
  UnderlayMessage m;

  while (reader->take_next_sample(reinterpret_cast<void*>(&m), &m_info) ==
         eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
    if (m_info.valid_data) {
      char* ptr = reinterpret_cast<char*>(
          &m_info.related_sample_identity.writer_guid());
      Identity sender_id(false);
      sender_id.set_data(ptr);
      msg_info_.set_sender_id(sender_id);

      Identity spare_id(false);
      spare_id.set_data(ptr + ID_SIZE);
      msg_info_.set_spare_id(spare_id);

      msg_info_.set_seq_num(m.seq());
      msg_info_.set_send_time(m.timestamp());
      // callback
      callback_(std::make_shared<std::string>(m.data()), channel_id, msg_info_);
    } else {
      AERROR << "Remote writer for topic "
             << reader->get_topicdescription()->get_name() << " is dead";
    }
  }
}

void SubscriberListener::on_subscription_matched(
    eprosima::fastdds::dds::DataReader* reader,
    const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) {
  (void)reader;
  (void)info;
}

}  // namespace dispatcher
}  // namespace transport
}  // namespace cyber
}  // namespace apollo
