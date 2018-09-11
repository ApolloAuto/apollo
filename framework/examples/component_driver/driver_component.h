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
#ifndef CYBERTRON_EXAMPLES_COMPONENT_DRIVER_COMPONENT_H
#define CYBERTRON_EXAMPLES_COMPONENT_DRIVER_COMPONENT_H

#include "cybertron/message/raw_message.h"
#include "cybertron/component/component.h"
#include "cybertron/component/timer_component.h"
#include "cybertron/proto/driver.pb.h"
#include "cybertron/scheduler/task.h"

namespace apollo {
namespace cybertron {
template <typename T>
class Writer;
}  // namespace cybertron
}  // namespace apollo

using apollo::cybertron::ComponentBase;
using apollo::cybertron::TimerComponent;
using apollo::cybertron::Writer;
using apollo::cybertron::message::RawMessage;
using apollo::cybertron::proto::CarStatus;
using apollo::cybertron::proto::Driver;

class DriverComponent : public TimerComponent {
 public:
  DriverComponent();
  bool Init() override;
  bool Proc() override;

 private:
  std::shared_ptr<Writer<Driver>> driver_writer_ = nullptr;
  std::shared_ptr<Writer<RawMessage>> dog_writer_ = nullptr;
  std::shared_ptr<apollo::cybertron::Task<CarStatus>> task_ = nullptr;
  std::shared_ptr<apollo::cybertron::Task<void>> void_task_ = nullptr;
};

CYBERTRON_REGISTER_COMPONENT(DriverComponent)
#endif
