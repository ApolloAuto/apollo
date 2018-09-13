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

#include "examples/component_watchdog/watchdog_component.h"
#include "cybertron/common/log.h"

using apollo::cybertron::Component;
using Food = apollo::cybertron::message::RawMessage;

WatchdogComponent::WatchdogComponent() {}

bool WatchdogComponent::Init() {
  DogConfig cfg;
  cfg.food = node_->Name();
  dog_ = std::make_shared<Watchdog>(cfg);
  return true;
}

bool WatchdogComponent::Proc(const std::shared_ptr<Food>& food) {
  auto condition = dog_->Feed(food->message);
  return condition == SurvivalCondition::ALIVE ? true : false;
}
