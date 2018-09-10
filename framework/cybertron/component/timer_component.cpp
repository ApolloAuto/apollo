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

#include "cybertron/component/timer_component.h"
#include "cybertron/timer/timer.h"

namespace apollo {
namespace cybertron {

TimerComponent::TimerComponent() {}

TimerComponent::~TimerComponent() {}

bool TimerComponent::Process() {
  // TODO(hewei03): Add some protection here.
  return Proc();
}

bool TimerComponent::Initialize(const TimerComponentConfig& config) {
  node_.reset(new Node(config.name()));
  if (config.has_config_file_path()) {
    config_file_path_ = config.config_file_path();
  }
  if (!Init()) {
    return false;
  }

  std::weak_ptr<TimerComponent> self =
      std::dynamic_pointer_cast<TimerComponent>(shared_from_this());
  auto func = [self]() {
    auto ptr = self.lock();
    if (ptr) {
      ptr->Proc();
    }
  };
  timer_.reset(new Timer(config.interval(), func, false));
  timer_->Start();
  return true;
}

}  // namespace cybertron
}  // namespace apollo
