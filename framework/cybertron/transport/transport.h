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
#include "cybertron/transport/lower_reach/hybrid_lower_reach.h"
#include "cybertron/transport/lower_reach/intra_lower_reach.h"
#include "cybertron/transport/lower_reach/lower_reach.h"
#include "cybertron/transport/lower_reach/rtps_lower_reach.h"
#include "cybertron/transport/lower_reach/shm_lower_reach.h"
#include "cybertron/transport/qos/qos_profile_conf.h"
#include "cybertron/transport/rtps/participant.h"
#include "cybertron/transport/upper_reach/hybrid_upper_reach.h"
#include "cybertron/transport/upper_reach/intra_upper_reach.h"
#include "cybertron/transport/upper_reach/rtps_upper_reach.h"
#include "cybertron/transport/upper_reach/shm_upper_reach.h"
#include "cybertron/transport/upper_reach/upper_reach.h"

namespace apollo {
namespace cybertron {
namespace transport {

using apollo::cybertron::proto::OptionalMode;

class Transport {
 public:
  Transport();
  virtual ~Transport();

  template <typename MessageT>
  static auto CreateUpperReach(const RoleAttributes& attr,
                               const OptionalMode& mode = OptionalMode::HYBRID)
      -> typename std::shared_ptr<UpperReach<MessageT>>;

  template <typename MessageT>
  static auto CreateLowerReach(
      const RoleAttributes& attr,
      const typename LowerReach<MessageT>::MessageListener& msg_listener,
      const OptionalMode& mode = OptionalMode::HYBRID) ->
      typename std::shared_ptr<LowerReach<MessageT>>;

  static ParticipantPtr participant() { return participant_; }

 private:
  static ParticipantPtr CreateParticipant();

  static ParticipantPtr participant_;
};

template <typename MessageT>
auto Transport::CreateUpperReach(const RoleAttributes& attr,
                                 const OptionalMode& mode) ->
    typename std::shared_ptr<UpperReach<MessageT>> {
  std::shared_ptr<UpperReach<MessageT>> upper_reach = nullptr;
  RoleAttributes modified_attr = attr;
  if (!modified_attr.has_qos_profile()) {
    modified_attr.mutable_qos_profile()->CopyFrom(
        QosProfileConf::QOS_PROFILE_DEFAULT);
  }
  switch (mode) {
    case OptionalMode::INTRA:
      upper_reach = std::make_shared<IntraUpperReach<MessageT>>(modified_attr);
      break;

    case OptionalMode::SHM:
      upper_reach = std::make_shared<ShmUpperReach<MessageT>>(modified_attr);
      break;

    case OptionalMode::RTPS:
      upper_reach = std::make_shared<RtpsUpperReach<MessageT>>(modified_attr,
                                                               participant_);
      break;

    default:
      upper_reach = std::make_shared<HybridUpperReach<MessageT>>(modified_attr,
                                                                 participant_);
      break;
  }

  RETURN_VAL_IF_NULL(upper_reach, nullptr);
  if (mode != OptionalMode::HYBRID) {
    upper_reach->Enable();
  }
  return upper_reach;
}

template <typename MessageT>
auto Transport::CreateLowerReach(
    const RoleAttributes& attr,
    const typename LowerReach<MessageT>::MessageListener& msg_listener,
    const OptionalMode& mode) ->
    typename std::shared_ptr<LowerReach<MessageT>> {
  std::shared_ptr<LowerReach<MessageT>> lower_reach = nullptr;
  RoleAttributes modified_attr = attr;
  if (!modified_attr.has_qos_profile()) {
    modified_attr.mutable_qos_profile()->CopyFrom(
        QosProfileConf::QOS_PROFILE_DEFAULT);
  }
  switch (mode) {
    case OptionalMode::INTRA:
      lower_reach = std::make_shared<IntraLowerReach<MessageT>>(modified_attr,
                                                                msg_listener);
      break;

    case OptionalMode::SHM:
      lower_reach = std::make_shared<ShmLowerReach<MessageT>>(modified_attr,
                                                              msg_listener);
      break;

    case OptionalMode::RTPS:
      lower_reach = std::make_shared<RtpsLowerReach<MessageT>>(modified_attr,
                                                               msg_listener);
      break;

    default:
      lower_reach = std::make_shared<HybridLowerReach<MessageT>>(
          modified_attr, msg_listener, participant_);
      break;
  }

  RETURN_VAL_IF_NULL(lower_reach, nullptr);
  if (mode != OptionalMode::HYBRID) {
    lower_reach->Enable();
  }
  return lower_reach;
}

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_TRANSPORT_H_
