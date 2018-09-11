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
#include "examples/component_perception/perception_component.h"

//#include "examples/component/perception.h"

using apollo::cybertron::proto::Driver;
using apollo::cybertron::proto::Perception;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;

namespace test {
PerceptionComponent::PerceptionComponent() {}

bool PerceptionComponent::Init() {
  RoleAttributes attr;
  std::string channel_per = "/perception/channel";
  attr.set_channel_name(channel_per);
  perception_writer_ = node_->CreateWriter<Perception>(attr);
  if (perception_writer_ == nullptr) {
    AERROR << "Perception create writer failed." << std::endl;
    return false;
  }

  attr.set_channel_name("/food");
  dog_writer_ = node_->CreateWriter<RawMessage>(attr);
  if (dog_writer_ == nullptr) {
    AERROR << "Driver create writer failed.";
    return false;
  }

  return true;
}

bool PerceptionComponent::Proc(const std::shared_ptr<RawMessage>& msg_str) {
  auto time_now = std::chrono::steady_clock::now();
  Driver msg;
  msg.ParseFromString(msg_str->message);
  AINFO << "Start Perception [" << msg.msg_id() << "]";
  /*
  if (TestCUDA() == 0) {
    AINFO << "Finish Perception [" << msg.msg_id() << "]";
  }
  */
  auto out_msg = std::make_shared<Perception>();
  out_msg->set_msg_id(msg.msg_id());
  out_msg->set_result(0);
  perception_writer_->Write(out_msg);

  auto food = std::make_shared<RawMessage>("milk");
  dog_writer_->Write(food);

  return true;
}
}
