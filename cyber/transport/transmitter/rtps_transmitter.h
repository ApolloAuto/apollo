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

#ifndef CYBER_TRANSPORT_TRANSMITTER_RTPS_TRANSMITTER_H_
#define CYBER_TRANSPORT_TRANSMITTER_RTPS_TRANSMITTER_H_

#include <memory>
#include <string>

#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/transport/rtps/attributes_filler.h"
#include "cyber/transport/rtps/participant.h"
#include "cyber/transport/transmitter/transmitter.h"
#include "fastrtps/Domain.h"
#include "fastrtps/attributes/PublisherAttributes.h"
#include "fastrtps/participant/Participant.h"
#include "fastrtps/publisher/Publisher.h"

namespace apollo {
namespace cyber {
namespace transport {

template <typename M>
class RtpsTransmitter : public Transmitter<M> {
 public:
  using MessagePtr = std::shared_ptr<M>;

  RtpsTransmitter(const RoleAttributes& attr,
                  const ParticipantPtr& participant);
  virtual ~RtpsTransmitter();

  void Enable() override;
  void Disable() override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

 private:
  bool Transmit(const M& msg, const MessageInfo& msg_info);

  ParticipantPtr participant_;
  eprosima::fastrtps::Publisher* publisher_;
};

template <typename M>
RtpsTransmitter<M>::RtpsTransmitter(const RoleAttributes& attr,
                                    const ParticipantPtr& participant)
    : Transmitter<M>(attr), participant_(participant), publisher_(nullptr) {}

template <typename M>
RtpsTransmitter<M>::~RtpsTransmitter() {
  Disable();
}

template <typename M>
void RtpsTransmitter<M>::Enable() {
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

template <typename M>
void RtpsTransmitter<M>::Disable() {
  if (this->enabled_) {
    publisher_ = nullptr;
    this->enabled_ = false;
  }
}

template <typename M>
bool RtpsTransmitter<M>::Transmit(const MessagePtr& msg,
                                  const MessageInfo& msg_info) {
  return Transmit(*msg, msg_info);
}

template <typename M>
bool RtpsTransmitter<M>::Transmit(const M& msg, const MessageInfo& msg_info) {
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

  if (participant_->is_shutdown()) {
    return false;
  }
  return publisher_->write(reinterpret_cast<void*>(&m), wparams);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_TRANSMITTER_RTPS_TRANSMITTER_H_
