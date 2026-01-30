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
 * @brief Defines the PIDPlusController class.
 */

#pragma once

#include "modules/control/control_component/controller_task_base/common/pid_controller.h"

#include "modules/control/control_task_base_extend/proto/pid_plus_conf.pb.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class PIDPlusController
 * @brief A proportional-integral-derivative controller for speed and steering
 using defualt integral hold
 */
class PIDPlusController : public PIDController {
public:
    /**
     * @brief initialize pid controller
     * @param pid_conf configuration for pid controller
     */
    void InitPID();
    /**
     * @brief set pid controller coefficients for the proportional,
     * integral, and derivative
     * @param Pidconf and speed
     */
    void SetPID(const PidPlusConf &pid_plus_conf);
};

}  // namespace control
}  // namespace apollo
