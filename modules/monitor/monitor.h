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

#ifndef MODULES_MONITOR_MONITOR_H_
#define MODULES_MONITOR_MONITOR_H_

#include <memory>
#include <string>
#include <vector>

#include "cybertron/component/timer_component.h"
#include "cybertron/cybertron.h"
#include "modules/monitor/common/recurrent_runner.h"
#include "modules/monitor/proto/system_status.pb.h"

/**
 * @namespace apollo::monitor
 * @brief apollo::monitor
 */
namespace apollo {
namespace monitor {

class Monitor : public apollo::cybertron::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
 private:
  std::vector<std::unique_ptr<RecurrentRunner>> runners_;
};

CYBERTRON_REGISTER_COMPONENT(Monitor)

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_MONITOR_H_
