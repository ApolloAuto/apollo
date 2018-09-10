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
#ifndef CYBERTRON_EXAMPLES_CYBERTRON_COMPONENT_H
#define CYBERTRON_EXAMPLES_CYBERTRON_COMPONENT_H

#include "cybertron/component/component.h"
#include "cybertron/proto/driver.pb.h"
#include "cybertron/proto/perception.pb.h"
#include "cybertron/message/message_traits.h"
#include "cybertron/message/raw_message.h"

namespace apollo {
namespace cybertron {
namespace transport {
template <typename T>
class Writer;
}
}
}

using apollo::cybertron::proto::Driver;
using apollo::cybertron::proto::Perception;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::cybertron::transport::Writer;

class ReceiverComponent : public Component<RawMessage> {
 public:
  ReceiverComponent();
  bool Init() override;
  bool Proc(const std::shared_ptr<RawMessage>& msg) override;
};

CYBERTRON_REGISTER_COMPONENT(ReceiverComponent)
#endif
