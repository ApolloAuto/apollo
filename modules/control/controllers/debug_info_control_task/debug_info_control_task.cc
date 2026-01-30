/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/controllers/debug_info_control_task/debug_info_control_task.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::control::ControlDebugInfo;

DebugInfoControlTask::DebugInfoControlTask() : name_("Debug Info Control Task") {
    AINFO << "Using " << name_;
}

Status DebugInfoControlTask::Init(std::shared_ptr<DependencyInjector> injector) {
    // hook: Apollo License Verification: v_apollo_park
    injector_ = injector;
    node_ = ::apollo::cyber::CreateNode("control_debug");
    control_debug_info_writer_
            = node_->CreateWriter<::apollo::control::ControlDebugInfo>(FLAGS_control_debug_info_topic);
    ACHECK(control_debug_info_writer_ != nullptr);
    return Status::OK();
}

Status DebugInfoControlTask::ComputeControlCommand(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis,
        const planning::ADCTrajectory *planning_published_trajectory,
        ControlCommand *cmd) {
    control_debug_info_msg_.Clear();
    if (injector_->control_process() && FLAGS_publish_control_debug_info) {
        control_debug_info_msg_.CopyFrom(injector_->control_debug_info());
        common::util::FillHeader(node_->Name(), &control_debug_info_msg_);
        if (FLAGS_sim_by_record) {
            control_debug_info_msg_.mutable_header()->set_timestamp_sec(chassis->header().timestamp_sec());
        }
        ADEBUG << "control debug msg is: " << control_debug_info_msg_.ShortDebugString();
        control_debug_info_writer_->Write(control_debug_info_msg_);
    }
    return Status::OK();
}

void DebugInfoControlTask::Stop() {}

Status DebugInfoControlTask::Reset() {
    control_debug_info_msg_.Clear();
    return Status::OK();
}

std::string DebugInfoControlTask::Name() const {
    return name_;
}

}  // namespace control
}  // namespace apollo
