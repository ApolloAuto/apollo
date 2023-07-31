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

#ifndef CYBER_COMPONENT_TIMER_COMPONENT_H_
#define CYBER_COMPONENT_TIMER_COMPONENT_H_

#include <memory>

#include "cyber/component/component_base.h"

namespace apollo {
namespace cyber {

class Timer;

/**
 * @brief .
 * TimerComponent is a timer component. Your component can inherit from
 * Component, and implement Init() & Proc(), They are called by the CyberRT
 * frame.
 */
class TimerComponent : public ComponentBase {
 public:
  TimerComponent();
  ~TimerComponent() override;

  /**
   * @brief init the component by protobuf object.
   *
   * @param config which is define in 'cyber/proto/component_conf.proto'
   *
   * @return returns true if successful, otherwise returns false
   */
  bool Initialize(const TimerComponentConfig& config) override;
  void Clear() override;
  bool Process();
  uint32_t GetInterval() const;

 private:
  /**
   * @brief The Proc logic of the component, which called by the CyberRT frame.
   *
   * @return returns true if successful, otherwise returns false
   */
  virtual bool Proc() = 0;

  uint32_t interval_ = 0;
  std::unique_ptr<Timer> timer_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMPONENT_TIMER_COMPONENT_H_
