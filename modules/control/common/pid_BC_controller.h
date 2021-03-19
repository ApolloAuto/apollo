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

#pragma once

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
 * @brief A proportional-integral-derivative controller for speed and steering
with backward-calculation-anti-windup
 */
class PIDBCController : public PIDController {
 public:
  /**
   * @brief compute control value based on the error,
   with backward-calculation-anti-windup
   * @param error error value, the difference between
   * a desired value and a measured value
   * @param dt sampling time interval
   * @return control value based on PID terms
   */
  virtual double Control(const double error, const double dt);

  virtual int OutputSaturationStatus();

 private:
};

}  // namespace control
}  // namespace apollo
