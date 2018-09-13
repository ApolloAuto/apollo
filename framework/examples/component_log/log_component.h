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
#ifndef CYBERTRON_EXAMPLES_CYBERTRON_LOG1_COMPONENT_H
#define CYBERTRON_EXAMPLES_CYBERTRON_LOG1_COMPONENT_H

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"
#include "cybertron/component/timer_component.h"

class LogComponent : public apollo::cybertron::TimerComponent {
 public:
  LogComponent();

  bool Init() override;
  bool Proc() override;
};

CYBERTRON_REGISTER_COMPONENT(LogComponent)
#endif
