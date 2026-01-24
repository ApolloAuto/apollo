/*****************************************************************************
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
 * @file speed_setting.cc
 **/

#include "modules/planning/traffic_rules/speed_setting/speed_setting.h"

#include "modules/common_msgs/external_command_msgs/speed_command.pb.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

SpeedSetting::SpeedSetting() : last_cruise_speed_(-1.0), last_sequence_num_(-1) {
    // hook: Apollo License Verification: v_apollo_park
}

Status SpeedSetting::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    const auto& planning_command = frame->local_view().planning_command;
    // If there is new planning command, use the last set cruise speed.
    if (!planning_command->has_header() || !planning_command->header().has_sequence_num()
        || planning_command->header().sequence_num() == last_sequence_num_) {
        // Set the cruise speed with last speed limit.
        if (last_cruise_speed_ > 0.0) {
            reference_line_info->SetCruiseSpeed(last_cruise_speed_);
        }
        return Status::OK();
    }
    // Update the new cruise speed when new motion command comes. The first time
    // received new motion command, use the new cruise speed.
    if (planning_command->is_motion_command()) {
        last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();
        return Status::OK();
    }

    // Only deal with planning command which has "SpeedCommand" as custom command.
    if (!planning_command->has_custom_command()
        || !planning_command->custom_command().Is<external_command::SpeedCommand>()) {
        // Set the cruise speed with last speed limit.
        if (last_cruise_speed_ > 0.0) {
            reference_line_info->SetCruiseSpeed(last_cruise_speed_);
        }
        return Status::OK();
    }
    const auto& custom_command = planning_command->custom_command();
    external_command::SpeedCommand speed_command;
    if (!custom_command.UnpackTo(&speed_command)) {
        // Set the cruise speed with last speed limit.
        if (last_cruise_speed_ > 0.0) {
            reference_line_info->SetCruiseSpeed(last_cruise_speed_);
        }
        AERROR << "Unpack speed command failed!";
        return Status(common::PLANNING_ERROR);
    }
    // If command is to restore the target speed, don't modify it in
    // "reference_line_info".
    if (speed_command.has_is_restore_target_speed() && speed_command.is_restore_target_speed()) {
        last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();
        return Status::OK();
    }
    last_sequence_num_ = planning_command->header().sequence_num();
    // Get the cruise speed for the first time.
    if (last_cruise_speed_ < 0.0) {
        last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();
    }
    // Deal with the command for adjusting target speed.
    if (speed_command.has_target_speed()) {
        reference_line_info->SetCruiseSpeed(speed_command.target_speed());
    } else if (speed_command.has_target_speed_factor()) {
        // Calculate the target speed with speed_factor.
        double target_speed = last_cruise_speed_ * speed_command.target_speed_factor();
        reference_line_info->SetCruiseSpeed(target_speed);
    }
    last_cruise_speed_ = reference_line_info->GetBaseCruiseSpeed();
    return Status::OK();
}

}  // namespace planning
}  // namespace apollo
