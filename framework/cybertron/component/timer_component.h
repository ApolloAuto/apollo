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

#ifndef CYBERTRON_COMPONENT_TIMER_COMPONENT_H_
#define CYBERTRON_COMPONENT_TIMER_COMPONENT_H_

#include <memory>

#include "cybertron/component/component_base.h"

namespace apollo {
namespace cybertron {

class Timer;

class TimerComponent : public ComponentBase {
 public:
  TimerComponent();
  ~TimerComponent() override;

  bool Initialize(const TimerComponentConfig& config) override;
  bool Process();

 protected:
  std::shared_ptr<Node> node_ = nullptr;

 private:
  virtual bool Proc() = 0;
  virtual bool Init() = 0;

  uint64_t interval_ = 0;
  std::unique_ptr<Timer> timer_;
};

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_COMPONENT_TIMER_COMPONENT_H_
