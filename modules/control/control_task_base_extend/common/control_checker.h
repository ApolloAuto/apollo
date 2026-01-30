/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
 * @file control_checker.h
 * @brief Defines the ControlChecker class.
 */

#pragma once

#include "modules/control/control_component/proto/check_status.pb.h"
#include "modules/control/control_task_base_extend/proto/safety_check_conf.pb.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class ControlChecker
 * @brief A proportional-integral-derivative controller for speed and steering
 using defualt integral hold
 */
class ControlChecker {
public:
    /**
     * @brief initialize pid controller
     * @param pid_conf configuration for pid controller
     */

    virtual ~ControlChecker() = default;

    static ControlCheckStatus check_lateral_error(const SafetyCheckConf& conf, double* lateral_error);
    static ControlCheckStatus check_heading_error(const SafetyCheckConf& conf, double* heading_error);
    static ControlCheckStatus check_station_error(const SafetyCheckConf& conf, double* station_error);
    static ControlCheckStatus check_speed_error(const SafetyCheckConf& conf, double* speed_error);
};

}  // namespace control
}  // namespace apollo
