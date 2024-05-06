/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 **/

#include "modules/planning/scenarios/pull_over/stage_retry_parking.h"

#include <memory>

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/pull_over/context.h"
#include "modules/planning/scenarios/pull_over/util.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

StageResult PullOverStageRetryParking::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: RetryParking";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "PullOverStageRetryParking planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }

  // set debug info in planning_data
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  auto* pull_over_debug = frame->mutable_open_space_info()
                              ->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_pull_over();
  pull_over_debug->mutable_position()->CopyFrom(pull_over_status.position());
  pull_over_debug->set_theta(pull_over_status.theta());
  pull_over_debug->set_length_front(pull_over_status.length_front());
  pull_over_debug->set_length_back(pull_over_status.length_back());
  pull_over_debug->set_width_left(pull_over_status.width_left());
  pull_over_debug->set_width_right(pull_over_status.width_right());
  frame->mutable_open_space_info()->sync_debug_instance();

  if (CheckADCPullOverOpenSpace()) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult PullOverStageRetryParking::FinishStage() {
  return FinishScenario();
}

bool PullOverStageRetryParking::CheckADCPullOverOpenSpace() {
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (!pull_over_status.has_position() ||
      !pull_over_status.position().has_x() ||
      !pull_over_status.position().has_y() || !pull_over_status.has_theta()) {
    ADEBUG << "pull_over status not set properly: "
           << pull_over_status.DebugString();
    return false;
  }

  const common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                            injector_->vehicle_state()->y()};
  const common::math::Vec2d target_position = {pull_over_status.position().x(),
                                               pull_over_status.position().y()};

  const double distance_diff = adc_position.DistanceTo(target_position);
  const double theta_diff = std::fabs(common::math::NormalizeAngle(
      pull_over_status.theta() - injector_->vehicle_state()->heading()));
  ADEBUG << "distance_diff[" << distance_diff << "] theta_diff[" << theta_diff
         << "]";
  const ScenarioPullOverConfig& scenario_config =
      GetContextAs<PullOverContext>()->scenario_config;
  // check distance/theta diff
  return (distance_diff <= scenario_config.max_distance_error_to_end_point() &&
          theta_diff <= scenario_config.max_theta_error_to_end_point());
}

}  // namespace planning
}  // namespace apollo
