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

/**
 * @file pid_controller.h
 * @brief Defines the PIDBCController class.
 */

#ifndef MODULES_CONTROL_COMMON_PID_BACKWARD_CACULATION_CONTROLLER_H_
#define MODULES_CONTROL_COMMON_PID_BACKWARD_CACULATION_CONTROLLER_H_

#include "modules/control/common/pid_controller.h"
#include "modules/control/proto/pid_conf.pb.h"
/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class PIDBCController
 * @brief A proportional–integral–derivative controller for speed and steering
with backward-caculation-anti-windup
 */
class PIDBCController : public PIDController {
 public:
  /**
   * @brief compute control value based on the error,
   with backward-caculation-anti-windup
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  virtual double Control(const double error, const double dt);

  /**
   * @brief get saturation status
   * @return saturation status
   */
  int SaturationStatus() const;

  /**
   * @brief get status that if integrator is hold
   * @return if integrator is hold return true
   */
  bool integrator_hold() const;

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kaw_ = 0.0;
  double previous_error_ = 0.0;
  double previous_error_i_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double saturation_high_ = 0.0;
  double saturation_low_ = 0.0;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int saturation_status_ = 0;
};

}  // namespace control
}  // namespace apollo

#endif  // MODULES_CONTROL_COMMON_PID_BACKWARD_CACULATION_CONTROLLER_H_
