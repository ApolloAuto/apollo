/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#ifndef MODULES_EXAMPLE_CYBERTRON_COMPONENT_H
#define MODULES_EXAMPLE_CYBERTRON_COMPONENT_H

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/message/message_traits.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/component/component.h"
#include "cybertron/proto/driver.pb.h"
#include "cybertron/proto/perception.pb.h"

namespace apollo {
namespace cybertron {
template <typename T>
class Writer;
}
}

using apollo::cybertron::proto::Driver;
using apollo::cybertron::proto::Perception;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::cybertron::Writer;

namespace test {
class PerceptionComponent : public Component<RawMessage> {
 public:
  PerceptionComponent();
  bool Init() override;
  bool Proc(const std::shared_ptr<RawMessage>& msg) override;

 private:
  std::shared_ptr<Writer<Perception>> perception_writer_ = nullptr;
  std::shared_ptr<Writer<RawMessage>> dog_writer_ = nullptr;
};

CYBERTRON_REGISTER_COMPONENT(PerceptionComponent)
}

#endif
