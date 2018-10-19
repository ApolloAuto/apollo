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

#ifndef CYBER_SERVICE_DISCOVERY_COMMUNICATION_PARTICIPANT_LISTENER_H_
#define CYBER_SERVICE_DISCOVERY_COMMUNICATION_PARTICIPANT_LISTENER_H_

#include <functional>
#include <mutex>

#include "fastrtps/Domain.h"
#include "fastrtps/participant/Participant.h"
#include "fastrtps/participant/ParticipantListener.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class ParticipantListener : public eprosima::fastrtps::ParticipantListener {
 public:
  using ChangeFunc = std::function<void(
      const eprosima::fastrtps::ParticipantDiscoveryInfo& info)>;

  explicit ParticipantListener(const ChangeFunc& callback);
  virtual ~ParticipantListener();

  virtual void onParticipantDiscovery(
      eprosima::fastrtps::Participant* p,
      eprosima::fastrtps::ParticipantDiscoveryInfo info);

 private:
  ChangeFunc callback_;
  std::mutex mutex_;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_DISCOVERY_COMMUNICATION_PARTICIPANT_LISTENER_H_
