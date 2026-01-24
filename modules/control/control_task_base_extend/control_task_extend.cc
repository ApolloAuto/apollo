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
 * @file
 * @brief Defines the Controller base class.
 */

#include "modules/control/control_task_base_extend/control_task_extend.h"

#include <memory>
#include <string>

#include "absl/strings/str_cat.h"

#include <cxxabi.h>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::planning::ADCTrajectory;

ControlTaskExtend::ControlTaskExtend() : name_("Control Task Base Extend") {
    AINFO << "Using " << name_ << ".";
}

bool ControlTaskExtend::LoadCalibrationTable(
        const std::string &calibration_table_path,
        calibration_table *calibration_table_conf) {
    std::string calibration_table_path_absolute
            = absl::StrCat("/apollo/modules/control/control_component/conf/", calibration_table_path);
    if (!apollo::cyber::common::GetProtoFromFile(calibration_table_path_absolute, calibration_table_conf)) {
        AERROR << "Load calibration table failed!";
        return false;
    }
    AINFO << "Load the calibraiton table file successfully, file path: " << calibration_table_path_absolute;
    return true;
}

bool ControlTaskExtend::VehicleStatusIdentificationUpdate(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis,
        const planning::ADCTrajectory *trajectory) {
    localization_ = localization;
    chassis_ = chassis;
    trajectory_message_ = trajectory;
    ACHECK(localization_ != nullptr) << "Fail to get localization msg!";
    ACHECK(chassis_ != nullptr) << "Fail to get chassis msg!";
    ACHECK(trajectory_message_ != nullptr) << "Fail to get trajectory msg!";

    IsForwardModel();
    IsEdgeFollow();

    return true;
}

bool ControlTaskExtend::IsForwardModel() {
    bool is_forward = false;
    if (chassis_->gear_location() != trajectory_message_->gear()) {
        if (chassis_->gear_location() != canbus::Chassis::GEAR_REVERSE) {
            is_forward = true;
        } else {
            is_forward = false;
        }
    } else {
        if (trajectory_message_->gear() != canbus::Chassis::GEAR_REVERSE) {
            is_forward = true;
        } else {
            is_forward = false;
        }
    }
    return is_forward;
}

bool ControlTaskExtend::IsEdgeFollow() {
    bool is_in_edge_follow = false;
    if (trajectory_message_->trajectory_type() == ADCTrajectory::EDGE_FOLLOW) {
        is_in_edge_follow = true;
    }
    return is_in_edge_follow;
}

bool ControlTaskExtend::CheckInPit(
        double pit_replan_check_time,
        double pit_replan_check_count,
        double vehicle_speed,
        bool replan) {
    static std::pair<double, int> replan_count(0, 0);

    double now = apollo::cyber::Clock::NowInSeconds();

    if (now - replan_count.first > pit_replan_check_time) {
        replan_count.first = 0;
        replan_count.second = 0;
    }

    if (replan) {
        if (now - replan_count.first > pit_replan_check_time) {
            replan_count.first = now;
            replan_count.second = 1;
        } else {
            replan_count.first = now;
            replan_count.second++;
        }
    }

    const auto &vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();
    if (replan_count.second >= pit_replan_check_count
        && abs(vehicle_speed) < vehicle_param.max_abs_speed_when_stopped()) {
        return true;
    } else {
        return false;
    }
}

std::string ControlTaskExtend::Name() const {
    return name_;
}

bool ControlTaskExtend::IsLargeCurvature(
        const double ref_curvature,
        const double min_large_ref_curvature,
        bool *is_in_large_curvature) {
    ADEBUG << "Into IsLargeCurvature. " << "ref_curvature is " << ref_curvature << ", min_large_ref_curvature is "
           << min_large_ref_curvature;
    bool is_in_large_cur = false;
    if (ref_curvature > min_large_ref_curvature) {
        ADEBUG << "ref_curvature is " << ref_curvature << ", in large curvature.";
        is_in_large_cur = true;
        *is_in_large_curvature = is_in_large_cur;
        return true;
    } else {
        is_in_large_cur = false;
        *is_in_large_curvature = is_in_large_cur;
        return false;
    }
}

bool ControlTaskExtend::IsLeftCurvature(const double ref_curvature) {
    bool is_left_cur = false;
    if (ref_curvature > 0) {
        is_left_cur = true;
    }
    return is_left_cur;
}

double ControlTaskExtend::GetVehicleSpeed(
        std::shared_ptr<DependencyInjector> injector,
        const bool &use_filter,
        const double &filter_coeff) {
    double vehicle_speed = injector->vehicle_state()->linear_velocity();
    if (use_filter) {
        // vehicle move direction changed, then reset filter
        if (vehicle_speed * vehicle_speed_last_ < 0) {
            vehicle_speed_last_ = 0.0;
        }
        vehicle_speed = ExponentialSmoothing::exponential_smoothing(vehicle_speed, vehicle_speed_last_, filter_coeff);
    }
    vehicle_speed_last_ = vehicle_speed;
    return vehicle_speed;
}

double ControlTaskExtend::ThrottleCmdFilter(
        const bool &use_filter,
        const double &filter_coeff,
        const double &throttle) {
    double throttle_filtered = throttle;
    if (use_filter) {
        throttle_filtered = ExponentialSmoothing::exponential_smoothing(throttle, throttle_last_, filter_coeff);
        throttle_last_ = throttle_filtered;
    }

    return throttle_filtered;
}


}  // namespace control
}  // namespace apollo
