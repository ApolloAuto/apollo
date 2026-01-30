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

#include "modules/control/controllers/replan_control_task/replan_control_task.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::control::ControlDebugInfo;
using apollo::control::ReplanRequestReasonCode;
using apollo::safety_manager::SafetyDecision;

ReplanControlTask::ReplanControlTask() : name_("Replan Control Task") {
    AINFO << "Using " << name_;
}

Status ReplanControlTask::Init(std::shared_ptr<DependencyInjector> injector) {
    // hook: Apollo License Verification: v_apollo_park
    injector_ = injector;
    node_ = ::apollo::cyber::CreateNode("replan_control");

    if (!ControlTask::LoadConfig<ReplanTaskConf>(&replan_task_conf_)) {
        AERROR << "Failed to replan control task config";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "Failed to load replan control task conf");
    }

    cyber::ReaderConfig safety_decision_reader_config;
    safety_decision_reader_config.channel_name = replan_task_conf_.guardian_decision_topic();
    safety_decision_reader_config.pending_queue_size = replan_task_conf_.guardian_decision_msg_pending_queue_size();

    safety_decision_reader_ = node_->CreateReader<SafetyDecision>(safety_decision_reader_config, nullptr);

    return Status::OK();
}

void ReplanControlTask::OnSafetyDecision(const std::shared_ptr<SafetyDecision> &safety_decision_msg) {
    ADEBUG << "Received safety decision data: run SafetyDecision callback.";
    std::lock_guard<std::mutex> lock(mutex_);
    decision_.CopyFrom(*safety_decision_msg);
}

Status ReplanControlTask::ComputeControlCommand(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis,
        const planning::ADCTrajectory *planning_published_trajectory,
        ControlCommand *cmd) {
    auto replan_debug_info = injector_->mutable_control_debug_info()->mutable_replan_debug();
    replan_debug_info->set_control_task_name(name_);
    replan_debug_info->set_trajectory_replan(planning_published_trajectory->is_replan());

    bool replan_request = false;
    ReplanRequestReasonCode replan_req_reason_code;
    std::string replan_request_reason = "";
    if (safety_decision_reader_ != nullptr) {
        ADEBUG << "Received safety decision data.";
        // get safety decision
        safety_decision_reader_->Observe();
        const auto &safety_decision_msg = safety_decision_reader_->GetLatestObserved();
        if (safety_decision_msg == nullptr) {
            AERROR << "safety decision msg is not ready!";
        } else {
            ADEBUG << "safety_decision_msg is " << safety_decision_msg->ShortDebugString();
            OnSafetyDecision(safety_decision_msg);
        }
        if (decision_.has_safety_mode_trigger_time()) {
            ADEBUG << "Safety mode triggered, need to replan.";
            replan_request = true;
            replan_req_reason_code = ReplanRequestReasonCode::REPLAN_REQ_ALL_REPLAN;
            replan_request_reason = "Safety function mode triggered, need to replan.";
        }
    } else {
        AERROR << "safety decision reader is not ready!";
    }

    if (chassis->error_code() == Chassis::CMD_NOT_IN_PERIOD) {
        ADEBUG << "chassis safety triggered, need to replan.";
        replan_request = true;
        replan_req_reason_code = ReplanRequestReasonCode::REPLAN_REQ_ALL_REPLAN;
        replan_request_reason = "Chassis safety mode triggered, need to replan.";
    }

    auto control_interactive_info = injector_->mutable_control_interactive_info();
    control_interactive_info->set_replan_request(replan_request);

    if (replan_request) {
        control_interactive_info->set_replan_req_reason_code(replan_req_reason_code);
        control_interactive_info->set_replan_request_reason(replan_request_reason);
    } else {
        control_interactive_info->set_replan_req_reason_code(ReplanRequestReasonCode::NONE_REPLAN);
        control_interactive_info->set_replan_request_reason("No triggered, do not need to replan");
    }

    replan_debug_info->set_is_need_replan(replan_request);

    return Status::OK();
}

void ReplanControlTask::Stop() {}

Status ReplanControlTask::Reset() {
    return Status::OK();
}

std::string ReplanControlTask::Name() const {
    return name_;
}

}  // namespace control
}  // namespace apollo
