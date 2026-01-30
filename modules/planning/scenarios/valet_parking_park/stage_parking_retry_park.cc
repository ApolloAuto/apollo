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
#include "modules/planning/scenarios/valet_parking_park/stage_parking_retry_park.h"
#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

StageResult StageParkingRetryPark::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  if (arrive_parking_spot_) {
    AINFO << "Stage Parking finish";
    frame->mutable_open_space_info()->set_openspace_planning_finish(true);
    return StageResult(StageStatusType::FINISHED);
  }
  auto scenario_context = GetContextAs<ValetParkingContext>();
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "StageParking planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }
	if (frame->open_space_info().destination_reached()) {
		frame->mutable_open_space_info()->set_openspace_planning_finish(true);
    CheckParkingAccuracy(frame);
    arrive_parking_spot_ = true;
		return result.SetStageStatus(StageStatusType::FINISHED);
	}
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageParkingRetryPark::FinishStage() {
  return StageResult(StageStatusType::FINISHED);
}

void StageParkingRetryPark::CheckParkingAccuracy(Frame* frame) {
  double ego_x = injector_->vehicle_state()->x();
  double ego_y = injector_->vehicle_state()->y();
  double ego_theta = injector_->vehicle_state()->heading();
  OpenSpaceTrajectoryOptimizerUtil::PathPointNormalizing(
      frame->open_space_info().origin_heading(),
      frame->open_space_info().origin_point(),
      &ego_x, &ego_y, &ego_theta);
  const std::vector<double>& end_pose =
      frame->open_space_info().open_space_end_pose();
  double delta_theta = fabs(
      common::math::NormalizeAngle(ego_theta - end_pose[2]));
  common::math::Vec2d vec(ego_x - end_pose[0], ego_y - end_pose[1]);
  common::math::Vec2d park_unit_vec = common::math::Vec2d::CreateUnitVec2d(end_pose[2]);
  double lat_error = park_unit_vec.CrossProd(vec);
  double lon_error = park_unit_vec.InnerProd(vec);
  AINFO << "lat error: " << lat_error << " "
        << "lon error: " << lon_error << " "
        << "delta_theta: " << delta_theta;
}

}  // namespace planning
}  // namespace apollo
