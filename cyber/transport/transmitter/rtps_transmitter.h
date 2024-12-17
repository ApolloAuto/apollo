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

#include <cstddef>
#include <memory>
#include <string>

#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/statistics/statistics.h"
#include "cyber/time/time.h"
#include "cyber/transport/dispatcher/subscriber_listener.h"
#include "cyber/transport/rtps/attributes_filler.h"
#include "cyber/transport/rtps/participant.h"
#include "cyber/transport/rtps/publisher.h"
#include "cyber/transport/transmitter/transmitter.h"

namespace apollo {
namespace cyber {
namespace transport {

template <typename M>
class RtpsTransmitter : public Transmitter<M> {
 public:
  using MessagePtr = std::shared_ptr<M>;
  using ParticipantPtr = std::shared_ptr<Participant>;

  RtpsTransmitter(const RoleAttributes& attr,
                  const ParticipantPtr& participant);
  virtual ~RtpsTransmitter();

  void Enable() override;
  void Disable() override;

  void Enable(const RoleAttributes& opposite_attr) override;
  void Disable(const RoleAttributes& opposite_attr) override;

  bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

  bool AcquireMessage(std::shared_ptr<M>& msg);

 private:
  bool Transmit(const M& msg, const MessageInfo& msg_info);

  ParticipantPtr participant_;
  PublisherPtr publisher_;
};

template <typename M>
bool RtpsTransmitter<M>::AcquireMessage(std::shared_ptr<M>& msg) {
  return false;
}

template <typename M>
RtpsTransmitter<M>::RtpsTransmitter(const RoleAttributes& attr,
                                    const ParticipantPtr& participant)
    : Transmitter<M>(attr), participant_(participant), publisher_(nullptr) {}

template <typename M>
RtpsTransmitter<M>::~RtpsTransmitter() {
  Disable();
}

template <typename M>
void RtpsTransmitter<M>::Enable(const RoleAttributes& opposite_attr) {
  (void)opposite_attr;
  this->Enable();
}

template <typename M>
void RtpsTransmitter<M>::Disable(const RoleAttributes& opposite_attr) {
  (void)opposite_attr;
  this->Disable();
}

template <typename M>
void RtpsTransmitter<M>::Enable() {
  if (this->enabled_) {
    return;
  }

  RETURN_IF_NULL(participant_);

  publisher_ = participant_->CreatePublisher(this->attr_.channel_name(),
                                             this->attr_.qos_profile());

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
    ADEBUG << "RtpsTransmitter not enable.";
    return false;
  }

  UnderlayMessage m;
  RETURN_VAL_IF(!message::SerializeToString(msg, &m.data()), false);
  m.timestamp(msg_info.send_time());
  m.seq(msg_info.seq_num());

  if (participant_->is_shutdown()) {
    return false;
  }

  RETURN_VAL_IF_NULL(publisher_, NULL);
  return publisher_->Write(m, msg_info);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TRANSPORT_TRANSMITTER_RTPS_TRANSMITTER_H_
