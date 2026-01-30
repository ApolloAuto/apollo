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

#include "modules/control/control_task_base_extend/common/pid_plus_controller.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace control {

void PIDPlusController::InitPID() {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_saturation_status_ = 0;
    integrator_hold_ = false;
    output_saturation_status_ = 0;
}

void PIDPlusController::SetPID(const PidPlusConf &pid_plus_conf) {
    kp_ = pid_plus_conf.pid_conf().kp();
    ki_ = pid_plus_conf.pid_conf().ki();
    kd_ = pid_plus_conf.pid_conf().kd();
    kaw_ = pid_plus_conf.pid_conf().kaw();
    integrator_enabled_ = pid_plus_conf.pid_conf().integrator_enable();
    integrator_saturation_high_ = std::fabs(pid_plus_conf.pid_conf().integrator_saturation_level());
    integrator_saturation_low_ = -std::fabs(pid_plus_conf.pid_conf().integrator_saturation_level());
    output_saturation_high_ = std::fabs(pid_plus_conf.pid_conf().output_saturation_level());
    output_saturation_low_ = -std::fabs(pid_plus_conf.pid_conf().output_saturation_level());
    ADEBUG << "kp: " << kp_ << " ki: " << ki_ << " kd: " << kd_ << " integrator_enabled: " << integrator_enabled_
           << " integrator_saturation_high: " << integrator_saturation_high_;
}

}  // namespace control
}  // namespace apollo
