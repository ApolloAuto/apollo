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
#include "modules/control/control_task_base_extend/common/control_checker.h"
#include "modules/common/math/math_utils.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace apollo {
namespace control {

ControlCheckStatus ControlChecker::check_lateral_error(const SafetyCheckConf& conf, double* lateral_error) {
    // if (!ControlUtil::is_need_safety_check()) {
    //     return NONE;
    // }
    double max_lateral_error_E = conf.check_threshold().max_lateral_error_e();
    double max_lateral_error_W = conf.check_threshold().max_lateral_error_w();
    // lateral error e check
    if (conf.use_lateral_error_e_check()) {
        if (std::fabs(*lateral_error) > max_lateral_error_E) {
            ADEBUG << "[Error] lateral error is not reliable due to: |" << *lateral_error << "| > "
                   << max_lateral_error_E;
            *lateral_error = common::math::Clamp(*lateral_error, -max_lateral_error_E, max_lateral_error_E);
            ADEBUG << "[Error] control lateral e failed.";
            return ERROR;
        }
    } else {
        ADEBUG << "control lateral error e check turn off.";
    }
    // lateral error warning check
    if (conf.use_lateral_error_w_check()) {
        if (std::fabs(*lateral_error) > max_lateral_error_W) {
            ADEBUG << "[Warning] lateral error is not reliable due to: |" << *lateral_error << "| > "
                   << max_lateral_error_W;
            ADEBUG << "[Warning] control lateral w failed.";
            return WARNING;
        }
    } else {
        ADEBUG << "control lateral error w check turn off.";
    }

    return NONE;
}

ControlCheckStatus ControlChecker::check_heading_error(const SafetyCheckConf& conf, double* heading_error) {
    // if (!ControlUtil::is_need_safety_check()) {
    //     return NONE;
    // }
    double max_heading_error_E = conf.check_threshold().max_heading_error_e();
    double max_heading_error_W = conf.check_threshold().max_heading_error_w();
    // heading error error check
    if (conf.use_heading_error_e_check()) {
        if (std::fabs(*heading_error) > max_heading_error_E) {
            ADEBUG << "[Error] heading error is not reliable due to: |" << *heading_error << "| > "
                   << max_heading_error_E;
            *heading_error = common::math::Clamp(*heading_error, -max_heading_error_E, max_heading_error_E);
            ADEBUG << "[Error] control heading e failed.";
            return ERROR;
        }
    } else {
        ADEBUG << "control heading error e check turn off.";
    }
    // heading error warning check
    if (conf.use_heading_error_w_check()) {
        if (std::fabs(*heading_error) > max_heading_error_W) {
            ADEBUG << "[Warning] heading error is not reliable due to: |" << *heading_error << "| > "
                   << max_heading_error_W;
            ADEBUG << "[Warning] control heading w failed.";
            return WARNING;
        }
    } else {
        ADEBUG << "control heading error w check turn off.";
    }

    return NONE;
}

ControlCheckStatus ControlChecker::check_station_error(const SafetyCheckConf& conf, double* station_error) {
    // if (!ControlUtil::is_need_safety_check()) {
    //     return NONE;
    // }
    double max_station_error_E = conf.check_threshold().max_station_error_e();
    double max_station_error_W = conf.check_threshold().max_station_error_w();
    // station error error check
    if (conf.use_station_error_e_check()) {
        if (std::fabs(*station_error) > max_station_error_E) {
            ADEBUG << "[Error] station error is not reliable due to: |" << *station_error << "| > "
                   << max_station_error_E;
            *station_error = common::math::Clamp(*station_error, -max_station_error_E, max_station_error_E);
            ADEBUG << "[Error] control station e failed.";
            return ERROR;
        }
    } else {
        ADEBUG << "control station error e check turn off.";
    }
    // station error warning check
    if (conf.use_station_error_w_check()) {
        if (std::fabs(*station_error) > max_station_error_W) {
            ADEBUG << "[Warning] station error is not reliable due to: |" << *station_error << "| > "
                   << max_station_error_W;
            ADEBUG << "[Warning] control station w failed.";
            return WARNING;
        }
    } else {
        ADEBUG << "control station error w check turn off.";
    }

    return NONE;
}

ControlCheckStatus ControlChecker::check_speed_error(const SafetyCheckConf& conf, double* speed_error) {
    // if (!ControlUtil::is_need_safety_check()) {
    //     return NONE;
    // }
    double max_speed_error_E = conf.check_threshold().max_speed_error_e();
    double max_speed_error_W = conf.check_threshold().max_speed_error_w();
    // speed error error check
    if (conf.use_speed_error_e_check()) {
        if (fabs(*speed_error) > max_speed_error_E) {
            ADEBUG << "[Error] speed error is not reliable due to: |" << *speed_error << "| > " << max_speed_error_E;
            *speed_error = common::math::Clamp(*speed_error, -max_speed_error_E, max_speed_error_E);
            ADEBUG << "[Error] control speed e failed.";
            return ERROR;
        }
    } else {
        ADEBUG << "control speed error e check turn off.";
    }
    // speed error warning check
    if (conf.use_speed_error_w_check()) {
        if (fabs(*speed_error) > max_speed_error_W) {
            ADEBUG << "[Warning] speed error is not reliable due to: |" << *speed_error << "| > " << max_speed_error_W;
            ADEBUG << "[Warning] control speed w failed.";
            return WARNING;
        }
    } else {
        ADEBUG << "control speed error w check turn off.";
    }

    return NONE;
}

}  // namespace control
}  // namespace apollo
