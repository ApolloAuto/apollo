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

#include "cyber/transport/rtps/sub_listener.h"

#include "cyber/common/log.h"
#include "cyber/common/util.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace transport {

SubListener::SubListener(const NewMsgCallback& callback)
    : callback_(callback) {}

SubListener::~SubListener() {}

void SubListener::onNewDataMessage(eprosima::fastrtps::Subscriber* sub) {
  RETURN_IF_NULL(sub);
  RETURN_IF_NULL(callback_);
  std::lock_guard<std::mutex> lock(mutex_);

  // fetch channel name
  auto channel_id = common::Hash(sub->getAttributes().topic.getTopicName());
  eprosima::fastrtps::SampleInfo_t m_info;
  UnderlayMessage m;

  RETURN_IF(!sub->takeNextData(reinterpret_cast<void*>(&m), &m_info));
  RETURN_IF(m_info.sampleKind != eprosima::fastrtps::ALIVE);

  // fetch MessageInfo
  char* ptr =
      reinterpret_cast<char*>(&m_info.related_sample_identity.writer_guid());
  Identity sender_id(false);
  sender_id.set_data(ptr);
  msg_info_.set_sender_id(sender_id);

  Identity spare_id(false);
  spare_id.set_data(ptr + ID_SIZE);
  msg_info_.set_spare_id(spare_id);

  uint64_t seq_num =
      ((int64_t)m_info.related_sample_identity.sequence_number().high) << 32 |
      m_info.related_sample_identity.sequence_number().low;
  msg_info_.set_seq_num(seq_num);

  // fetch message string
  std::shared_ptr<std::string> msg_str =
      std::make_shared<std::string>(m.data());

  uint64_t recv_time = Time::Now().ToNanosecond();
  uint64_t base_time = recv_time & 0xfffffff0000000;
  int32_t send_time_low = m.timestamp();
  uint64_t send_time = base_time | send_time_low;
  int32_t msg_seq_num = m.seq();

  msg_info_.set_msg_seq_num(msg_seq_num);
  msg_info_.set_send_time(send_time);

  // callback
  callback_(channel_id, msg_str, msg_info_);
}

void SubListener::onSubscriptionMatched(
    eprosima::fastrtps::Subscriber* sub,
    eprosima::fastrtps::MatchingInfo& info) {
  (void)sub;
  (void)info;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
