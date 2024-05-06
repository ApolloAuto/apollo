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

#include "modules/control/control_component/controller_task_base/common/pid_BC_controller.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace control {

double PIDBCController::Control(const double error, const double dt) {
  if (dt <= 0) {
    AWARN << "dt <= 0, will use the last output";
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }

  // backward calculation
  if (!integrator_enabled_) {
    integral_ = 0;
  } else {
    double u = error * kp_ + integral_ + error * dt * ki_ + diff * kd_;
    double aw_term = common::math::Clamp(u, output_saturation_high_,
                                         output_saturation_low_) -
                     u;
    if (aw_term > 1e-6) {
      output_saturation_status_ = -1;
    } else if (aw_term < -1e-6) {
      output_saturation_status_ = 1;
    } else {
      output_saturation_status_ = 0;
    }
    integral_ += kaw_ * aw_term + error * dt;
  }

  previous_error_ = error;
  output = common::math::Clamp(error * kp_ + integral_ + diff * kd_,
                               output_saturation_high_,
                               output_saturation_low_);  // Ki already applied
  previous_output_ = output;
  return output;
}

int PIDBCController::OutputSaturationStatus() {
  return output_saturation_status_;
}

}  // namespace control
}  // namespace apollo
