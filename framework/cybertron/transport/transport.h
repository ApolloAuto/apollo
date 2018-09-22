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

#ifndef CYBERTRON_TRANSPORT_TRANSPORT_H_
#define CYBERTRON_TRANSPORT_TRANSPORT_H_

#include <memory>
#include <string>

#include "cybertron/proto/transport_conf.pb.h"
#include "cybertron/transport/qos/qos_profile_conf.h"
#include "cybertron/transport/receiver/hybrid_receiver.h"
#include "cybertron/transport/receiver/intra_receiver.h"
#include "cybertron/transport/receiver/receiver.h"
#include "cybertron/transport/receiver/rtps_receiver.h"
#include "cybertron/transport/receiver/shm_receiver.h"
#include "cybertron/transport/rtps/participant.h"
#include "cybertron/transport/transmitter/hybrid_transmitter.h"
#include "cybertron/transport/transmitter/intra_transmitter.h"
#include "cybertron/transport/transmitter/rtps_transmitter.h"
#include "cybertron/transport/transmitter/shm_transmitter.h"
#include "cybertron/transport/transmitter/transmitter.h"

namespace apollo {
namespace cybertron {
namespace transport {

using apollo::cybertron::proto::OptionalMode;

class Transport {
 public:
  Transport();
  virtual ~Transport();

  template <typename M>
  static auto CreateTransmitter(const RoleAttributes& attr,
                                const OptionalMode& mode = OptionalMode::HYBRID)
      -> typename std::shared_ptr<Transmitter<M>>;

  template <typename M>
  static auto CreateReceiver(
      const RoleAttributes& attr,
      const typename Receiver<M>::MessageListener& msg_listener,
      const OptionalMode& mode = OptionalMode::HYBRID) ->
      typename std::shared_ptr<Receiver<M>>;

  static ParticipantPtr participant();

 private:
  static ParticipantPtr CreateParticipant();

  static ParticipantPtr participant_;
};

template <typename M>
auto Transport::CreateTransmitter(const RoleAttributes& attr,
                                  const OptionalMode& mode) ->
    typename std::shared_ptr<Transmitter<M>> {
  std::shared_ptr<Transmitter<M>> transmitter = nullptr;
  RoleAttributes modified_attr = attr;
  if (!modified_attr.has_qos_profile()) {
    modified_attr.mutable_qos_profile()->CopyFrom(
        QosProfileConf::QOS_PROFILE_DEFAULT);
  }
  switch (mode) {
    case OptionalMode::INTRA:
      transmitter = std::make_shared<IntraTransmitter<M>>(modified_attr);
      break;

    case OptionalMode::SHM:
      transmitter = std::make_shared<ShmTransmitter<M>>(modified_attr);
      break;

    case OptionalMode::RTPS:
      transmitter =
          std::make_shared<RtpsTransmitter<M>>(modified_attr, participant());
      break;

    default:
      transmitter =
          std::make_shared<HybridTransmitter<M>>(modified_attr, participant());
      break;
  }

  RETURN_VAL_IF_NULL(transmitter, nullptr);
  if (mode != OptionalMode::HYBRID) {
    transmitter->Enable();
  }
  return transmitter;
}

template <typename M>
auto Transport::CreateReceiver(
    const RoleAttributes& attr,
    const typename Receiver<M>::MessageListener& msg_listener,
    const OptionalMode& mode) -> typename std::shared_ptr<Receiver<M>> {
  std::shared_ptr<Receiver<M>> receiver = nullptr;
  RoleAttributes modified_attr = attr;
  if (!modified_attr.has_qos_profile()) {
    modified_attr.mutable_qos_profile()->CopyFrom(
        QosProfileConf::QOS_PROFILE_DEFAULT);
  }
  switch (mode) {
    case OptionalMode::INTRA:
      receiver =
          std::make_shared<IntraReceiver<M>>(modified_attr, msg_listener);
      break;

    case OptionalMode::SHM:
      receiver = std::make_shared<ShmReceiver<M>>(modified_attr, msg_listener);
      break;

    case OptionalMode::RTPS:
      receiver = std::make_shared<RtpsReceiver<M>>(modified_attr, msg_listener);
      break;

    default:
      receiver = std::make_shared<HybridReceiver<M>>(
          modified_attr, msg_listener, participant());
      break;
  }

  RETURN_VAL_IF_NULL(receiver, nullptr);
  if (mode != OptionalMode::HYBRID) {
    receiver->Enable();
  }
  return receiver;
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_TRANSPORT_H_
