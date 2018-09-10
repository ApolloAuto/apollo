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

#ifndef CYBERTRON_TRANSPORT_UPPER_REACH_RTPS_UPPER_REACH_H_
#define CYBERTRON_TRANSPORT_UPPER_REACH_RTPS_UPPER_REACH_H_

#include <memory>
#include <string>

#include <fastrtps/Domain.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/publisher/Publisher.h>

#include "cybertron/common/log.h"
#include "cybertron/message/message_traits.h"
#include "cybertron/transport/rtps/attributes_filler.h"
#include "cybertron/transport/rtps/participant.h"
#include "cybertron/transport/upper_reach/upper_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

template <typename MessageT>
class RtpsUpperReach : public UpperReach<MessageT> {
 public:
  using MessagePtr = std::shared_ptr<MessageT>;

  RtpsUpperReach(const RoleAttributes& attr,
                 const ParticipantPtr& participant);
  virtual ~RtpsUpperReach();

  void Enable() override;
  void Disable() override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

 private:
  bool Transmit(const MessageT& msg, const MessageInfo& msg_info);

  ParticipantPtr participant_;
  eprosima::fastrtps::Publisher* publisher_;
};

template <typename MessageT>
RtpsUpperReach<MessageT>::RtpsUpperReach(const RoleAttributes& attr,
                                         const ParticipantPtr& participant)
    : UpperReach<MessageT>(attr),
      participant_(participant),
      publisher_(nullptr) {}

template <typename MessageT>
RtpsUpperReach<MessageT>::~RtpsUpperReach() {
  Disable();
}

template <typename MessageT>
void RtpsUpperReach<MessageT>::Enable() {
  if (this->enabled_) {
    return;
  }

  RETURN_IF_NULL(participant_);

  eprosima::fastrtps::PublisherAttributes pub_attr;
  RETURN_IF(!AttributesFiller::FillInPubAttr(
      this->attr_.channel_name(), this->attr_.qos_profile(), &pub_attr));
  publisher_ = eprosima::fastrtps::Domain::createPublisher(
      participant_->fastrtps_participant(), pub_attr);
  RETURN_IF_NULL(publisher_);
  this->enabled_ = true;
}

template <typename MessageT>
void RtpsUpperReach<MessageT>::Disable() {
  if (this->enabled_) {
    publisher_ = nullptr;
    this->enabled_ = false;
  }
}

template <typename MessageT>
bool RtpsUpperReach<MessageT>::Transmit(const MessagePtr& msg,
                                        const MessageInfo& msg_info) {
  return Transmit(*msg, msg_info);
}

template <typename MessageT>
bool RtpsUpperReach<MessageT>::Transmit(const MessageT& msg,
                                        const MessageInfo& msg_info) {
  if (!this->enabled_) {
    ADEBUG << "not enable.";
    return false;
  }

  UnderlayMessage m;
  RETURN_VAL_IF(!message::SerializeToString(msg, &m.data()), false);

  eprosima::fastrtps::rtps::WriteParams wparams;

  char* ptr =
      reinterpret_cast<char*>(&wparams.related_sample_identity().writer_guid());

  memcpy(ptr, msg_info.sender_id().data(), ID_SIZE);
  memcpy(ptr + ID_SIZE, msg_info.spare_id().data(), ID_SIZE);

  wparams.related_sample_identity().sequence_number().high =
      (int32_t)((msg_info.seq_num() & 0xFFFFFFFF00000000) >> 32);
  wparams.related_sample_identity().sequence_number().low =
      (int32_t)(msg_info.seq_num() & 0xFFFFFFFF);

  return publisher_->write(reinterpret_cast<void*>(&m), wparams);
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_UPPER_REACH_RTPS_UPPER_REACH_H_
