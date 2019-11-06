/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/submodules/pid_lqr_controller_submodule.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {
PidLqrControllerSubmodule::PidLqrControllerSubmodule()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

PidLqrControllerSubmodule::~PidLqrControllerSubmodule() {}

std::string PidLqrControllerSubmodule::Name() const {
  return FLAGS_pid_lqr_controller_submodule_name;
}

bool PidLqrControllerSubmodule::Init() {
  /* LQR controller*/
  if (!cyber::common::GetProtoFromFile(FLAGS_lqr_controller_conf_file,
                                       &lqr_controller_conf_)) {
    AERROR << "Unable to load LQR controller conf file: " +
                  FLAGS_lqr_controller_conf_file;
    return false;
  }
  if (!lqr_controller_.Init(&lqr_controller_conf_).ok()) {
    monitor_logger_buffer_.ERROR(
        "Control init LQR controller failed! Stopping...");
    return false;
  }
  /* PID controller*/
  if (!cyber::common::GetProtoFromFile(FLAGS_pid_controller_conf_file,
                                       &pid_controller_conf_)) {
    AERROR << "Unable to load PID controller conf file: " +
                  FLAGS_pid_controller_conf_file;
    return false;
  }
  if (!pid_controller_.Init(&pid_controller_conf_).ok()) {
    monitor_logger_buffer_.ERROR(
        "Control init PID controller failed! Stopping...");
    return false;
  }
  return true;
}

}  // namespace control
}  // namespace apollo