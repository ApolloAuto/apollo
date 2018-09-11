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

#ifndef CYBERTRON_TRANSPORT_RTPS_PARTICIPANT_H_
#define CYBERTRON_TRANSPORT_RTPS_PARTICIPANT_H_

#include <memory>
#include <string>

#include <fastrtps/Domain.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/rtps/common/Locator.h>

#include "cybertron/transport/rtps/underlay_message_type.h"

namespace apollo {
namespace cybertron {
namespace transport {

class Participant;
using ParticipantPtr = std::shared_ptr<Participant>;

class Participant {
 public:
  Participant(const std::string& name, int send_port,
              eprosima::fastrtps::ParticipantListener* listener = nullptr);
  virtual ~Participant();

  void Shutdown();

  eprosima::fastrtps::Participant* fastrtps_participant() const {
    return fastrtps_participant_;
  }

 private:
  void CreateFastRtpsParticipant(
      const std::string& name, int send_port,
      eprosima::fastrtps::ParticipantListener* listener);

  bool shutdown_;
  UnderlayMessageType type_;
  eprosima::fastrtps::Participant* fastrtps_participant_;
};

}  // namespace transport
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TRANSPORT_RTPS_PARTICIPANT_H_
