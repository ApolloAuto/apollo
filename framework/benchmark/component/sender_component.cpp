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
#include "benchmark/component/sender_component.h"

#include <chrono>

#include "cybertron/common/log.h"
#include "cybertron/time/time.h"

using apollo::cybertron::proto::RoleAttributes;

SenderComponent::SenderComponent() {
}

bool SenderComponent::Init() {
  RoleAttributes attr;
  attr.set_channel_name("/channel0");
  sender_writer_ = node_->CreateWriter<Driver>(attr);
  if (sender_writer_ == nullptr) {
    AERROR << "Sender create writer failed.";
    return false;
  }

  attr.set_channel_name("/carstatus/channel");
  carstatus_writer_ = node_->CreateWriter<CarStatus>(attr);
  if (carstatus_writer_ == nullptr) {
    AERROR << "Driver create writer failed.";
    return false;
  }
  return true;
}

bool SenderComponent::Proc() {
  static int i = 0;
  auto out_msg = std::make_shared<Driver>();
  out_msg->set_msg_id(i++);
  auto t = std::chrono::steady_clock::now();
  out_msg->set_timestamp(apollo::cybertron::Time::Now().ToNanosecond());
  sender_writer_->Write(out_msg);

  static int j = 0;
  for (int n = 0; n < 10; ++n) {
    auto out_msg = std::make_shared<CarStatus>();
    out_msg->set_msg_id(j++);
    carstatus_writer_->Write(out_msg);
  }
  return true;
}
