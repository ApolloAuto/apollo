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
#include "benchmark/component/receiver_component.h"
#include "cybertron/time/time.h"

using apollo::cybertron::proto::Driver;
using apollo::cybertron::proto::Perception;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;

ReceiverComponent::ReceiverComponent() {
}

bool ReceiverComponent::Init() {
  return true;
}

bool ReceiverComponent::Proc(const std::shared_ptr<RawMessage>& msg_str) {
  uint64_t now = apollo::cybertron::Time::Now().ToNanosecond();
  Driver msg;
  msg.ParseFromString(msg_str->message);
  uint64_t diff = now - msg.timestamp();
  AINFO << "Duration(us): " << diff / 1000 << " Id: " << msg.msg_id();
  return true;
}
