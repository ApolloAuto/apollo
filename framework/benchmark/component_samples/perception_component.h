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
#ifndef CYBERTRON_BENCHMARK_COMPONENT_SAMPLES_PERCEPTION_COMPONENT_H
#define CYBERTRON_BENCHMARK_COMPONENT_SAMPLES_PERCEPTION_COMPONENT_H

#include "cybertron/component/component.h"
#include "benchmark/types.h"

using apollo::cybertron::component::Component;
using apollo::cybertron::component::ComponentConfig;
using apollo::cybertron::croutine::CRoutine;

extern int YIELD_TIMES;
extern int count[10];

class PerceptionComponent : public Component<Driver> {
 public:
  PerceptionComponent() {}
  int Init() {};
  bool Proc(const std::shared_ptr<Driver>& msg) override {
    for (int i = 0; i < YIELD_TIMES; ++i) {
      CRoutine::Yield();
    }
    count[msg->msg_id()] = 1;
  }
  
};

#endif
