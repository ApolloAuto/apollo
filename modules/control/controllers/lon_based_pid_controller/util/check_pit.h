/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include <utility>

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/controllers/lon_based_pid_controller/proto/lon_based_pid_controller_conf.pb.h"

#include "cyber/time/clock.h"

namespace apollo {
namespace control {

class CheckPit {
 public:
  static bool CheckInPit(SimpleLongitudinalDebug* debug,
                          const LonBasedPidControllerConf* conf,
                          double speed,
                          bool replan);
};

}  // namespace control
}  // namespace apollo
