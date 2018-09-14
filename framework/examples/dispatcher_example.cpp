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

#include <iostream>

#include "cybertron/dispatcher/dispatcher.h"
#include "cybertron/proto/component_config.pb.h"
#include "examples/component_perception/perception_component.h"

// To use this example, we need set run_mode to MODE_SIMULATION
// (run_mode is in file conf/cybertron.pb.conf)

int main(int argc, char* argv[]) {
  apollo::cybertron::proto::ComponentConfig config;
  config.set_name("perception");
  auto reader = config.add_readers();
  reader->set_channel("/driver/channel");

  auto perception = std::make_shared<::test::PerceptionComponent>();
  perception->Initialize(config);

  auto driver_msg = std::make_shared<apollo::cybertron::proto::Driver>();
  driver_msg->set_msg_id(0);
  driver_msg->set_timestamp(0);
  driver_msg->mutable_header()->set_timestamp(0);

  auto driver_raw_msg =
      std::make_shared<apollo::cybertron::message::RawMessage>();
  driver_msg->SerializeToString(&driver_raw_msg->message);

  perception->Proc(driver_raw_msg);

  auto message = apollo::cybertron::dispatcher::Dispatcher::Instance()
                     ->GetMessage<Perception>("/perception/channel");

  bool is_empty = message->IsPublishedEmpty();

  if (is_empty) {
    std::cout << "simulation failed." << std::endl;
  } else {
    std::cout << "simulation succ." << std::endl;
  }

  return 0;
}
