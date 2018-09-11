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
#include "examples/component_planning/planning_component.h"

using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::cybertron::proto::Perception;
using apollo::cybertron::proto::RoleAttributes;

PlanningComponent::PlanningComponent() {}

bool PlanningComponent::Init() {
  RoleAttributes attr;
  attr.set_channel_name("/food");
  dog_writer_ = node_->CreateWriter<RawMessage>(attr);
  if (dog_writer_ == nullptr) {
    AERROR << "Driver create writer failed.";
    return false;
  }
  return true;
}

bool PlanningComponent::Proc(const std::shared_ptr<Perception>& msg0,
                             const std::shared_ptr<CarStatus>& msg1) {
  AINFO << "Start Planning [" << msg0->msg_id() << "] [" << msg1->msg_id()
        << "]";
  double result = 0.0;
  for (int i = 0; i < 10000000; i++) {
    result += tan(
        atan(tan(atan(tan(atan(tan(atan(tan(atan(123456789.123456789))))))))));
  }

  auto food = std::make_shared<RawMessage>("egg");
  dog_writer_->Write(food);

  return true;
}
