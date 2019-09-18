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

#include "cyber/service_discovery/communication/participant_listener.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

ParticipantListener::ParticipantListener(const ChangeFunc& callback)
    : callback_(callback) {}

ParticipantListener::~ParticipantListener() {
  std::lock_guard<std::mutex> lck(mutex_);
  callback_ = nullptr;
}

void ParticipantListener::onParticipantDiscovery(
    eprosima::fastrtps::Participant* p,
    eprosima::fastrtps::ParticipantDiscoveryInfo info) {
  RETURN_IF_NULL(callback_);
  (void)p;
  std::lock_guard<std::mutex> lock(mutex_);
  callback_(info);
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
