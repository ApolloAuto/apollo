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
#ifndef CYBERTRON_EXAMPLES_COMPONENT_DRIVER_COMPONENT_H
#define CYBERTRON_EXAMPLES_COMPONENT_DRIVER_COMPONENT_H

#include "cybertron/component/timer_component.h"
#include "cybertron/proto/driver.pb.h"
#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"

namespace apollo {
namespace cybertron {
namespace transport {
template <typename T>
class Writer;
}
}
}

using apollo::cybertron::proto::Driver;
using apollo::cybertron::proto::CarStatus;
using apollo::cybertron::TimerComponent;
using apollo::cybertron::ComponentBase;
using apollo::cybertron::Writer;

class SenderComponent : public TimerComponent {
 public:
  SenderComponent();
  bool Init() override;
  bool Proc() override;

 private:
  std::shared_ptr<Writer<Driver>> sender_writer_ = nullptr;
  std::shared_ptr<Writer<CarStatus>> carstatus_writer_ = nullptr;
};

CYBERTRON_REGISTER_COMPONENT(SenderComponent)
#endif
