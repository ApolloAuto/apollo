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

#include "modules/planning/tasks/obstacle_nudge_decider/obstacle_nudge_decider.h"

#include <algorithm>
#include <memory>

#include "modules/common_msgs/planning_msgs/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool ObstacleNudgeDecider::Init(
        const std::string &config_dir,
        const std::string &name,
        const std::shared_ptr<DependencyInjector> &injector) {
    if (!Task::Init(config_dir, name, injector)) {
        return false;
    }
    // Load the config this task.
    if (!Task::LoadConfig<ObstacleNudgeDeciderConfig>(&config_)) {
        return false;
    }
    // reset NudgeCalculation
    nudge_calc_.reset(new NudgeCalculation(config_));

    return true;
}

Status ObstacleNudgeDecider::Execute(Frame *frame, ReferenceLineInfo *reference_line_info) {
    Task::Execute(frame, reference_line_info);
    return Process(frame, reference_line_info);
}

Status ObstacleNudgeDecider::Process(Frame *frame, ReferenceLineInfo *reference_line_info) {
    if (reference_line_info->index() == 0 && nullptr != injector_->frame_history()->Latest()) {
        ParkDataCenter::Instance()->UpdateHistoryNudgeInfo(injector_->frame_history()->Latest()->SequenceNum());
        AINFO << "Update history nudge info";
    }
    AINFO << "Task process: ObstacleNudgeDecider start.";
    if (!FLAGS_enable_nudge_decider || !config_.enbale_nugde_static_obs() || nullptr == reference_line_info
        || reference_line_info->path_reusable()) {
        AINFO << "path reusable" << reference_line_info->path_reusable() << ",skip";
        return Status::OK();
    }

    bool adc_in_nudge_state
            = injector_->planning_context()->planning_status().path_decider().is_in_path_lane_borrow_scenario();

    // update for nudge calc
    nudge_calc_->set_in_nudge_state(adc_in_nudge_state);

    // calc for nudge decision
    nudge_calc_->BuildNudgeDecisionWithObs(frame, injector_->frame_history()->Latest(), reference_line_info_);

    AINFO << "ObstacleNudgeDecider end";
    return Status::OK();
}

}  // namespace planning
}  // namespace apollo
