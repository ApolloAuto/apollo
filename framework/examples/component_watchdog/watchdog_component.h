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

#ifndef EXAMPLES_COMPONENT_WATCHDOG_WATCHDOG_COMPONENT_H_
#define EXAMPLES_COMPONENT_WATCHDOG_WATCHDOG_COMPONENT_H_

#include "cybertron/message/raw_message.h"
#include "cybertron/component/component.h"
#include "cybertron/time/time.h"
#include "examples/component_watchdog/watchdog.h"

using apollo::cybertron::Component;
using Food = apollo::cybertron::message::RawMessage;

class WatchdogComponent : public Component<Food> {
 public:
  WatchdogComponent();

  bool Init() override;
  bool Proc(const std::shared_ptr<Food>& food) override;

 private:
  std::shared_ptr<Watchdog> dog_ = nullptr;
};

CYBERTRON_REGISTER_COMPONENT(WatchdogComponent)

#endif  // EXAMPLES_COMPONENT_WATCHDOG_WATCHDOG_COMPONENT_H_
