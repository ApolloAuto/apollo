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

#include "cyber/transport/transport.h"

#include <atomic>
#include <mutex>

#include "cyber/common/global_data.h"

namespace apollo {
namespace cyber {
namespace transport {

static std::atomic<bool> shutdown_ = {false};
static std::mutex participant_mutex_;

ParticipantPtr Transport::participant_ = nullptr;

Transport::Transport() {}

Transport::~Transport() {}

void Transport::Shutdown() {
  if (shutdown_.exchange(true)) {
    return;
  }
  if (participant_ != nullptr) {
    participant_->Shutdown();
    participant_ = nullptr;
  }
}

ParticipantPtr Transport::CreateParticipant() {
  std::string participant_name =
      common::GlobalData::Instance()->HostName() + "+" +
      std::to_string(common::GlobalData::Instance()->ProcessId());
  return std::make_shared<Participant>(participant_name, 11512);
}

ParticipantPtr Transport::participant() {
  if (participant_ != nullptr) {
    return participant_;
  }
  std::lock_guard<std::mutex> lck(participant_mutex_);
  if (participant_ == nullptr) {
    participant_ = CreateParticipant();
  }
  return participant_;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
