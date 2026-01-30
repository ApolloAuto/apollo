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
#include <vector>
#include "modules/planning/planning_base/common/frame.h"

#include "modules/planning/scenarios/valet_parking_park/stage_parking_park.h"

namespace apollo {
namespace planning {

StageResult StageParkingPark::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
	AINFO << "new version of stage parking park";
  // Open space planning doesn't use planning_init_point from upstream because
  // of different stitching strategy
  auto scenario_context = GetContextAs<ValetParkingContext>();
  
  if (arrive_parking_spot_) {
    AINFO << "Stage Parking finish";
    frame->mutable_open_space_info()->set_openspace_planning_finish(true);
    return StageResult(StageStatusType::FINISHED);
  }

  scenario_config_ = scenario_context->scenario_config;
  frame->mutable_open_space_info()->set_is_on_open_space_trajectory(true);
  *(frame->mutable_open_space_info()->mutable_target_parking_spot_id()) =
      scenario_context->target_parking_spot_id;
  StageResult result = ExecuteTaskOnOpenSpace(frame);
  if (result.HasError()) {
    AERROR << "StageParking planning error";
    return result.SetStageStatus(StageStatusType::ERROR);
  }
  if (frame->open_space_info().destination_reached()) {
    arrive_parking_spot_ = true;
		if (!CheckADCParkingCompleted(frame)) {
			AINFO << "Stage Parking finish but do not reach parking spot,"
             "retry parking";
    	return FinishStage(frame);
		} else {
			frame->mutable_open_space_info()->set_openspace_planning_finish(true);
      AINFO << "Stage Parking finish";
      parking_mission_info_.status = ParkingMissionStatus::DONE;
			return result.SetStageStatus(StageStatusType::FINISHED);
		}
	}
  return result.SetStageStatus(StageStatusType::RUNNING);
}

StageResult StageParkingPark::FinishStage(Frame* frame) {
  next_stage_ = "VALET_PARKING_RETRY_PARK";
  frame->mutable_open_space_info()->
      mutable_optimizer_trajectory_data()->clear();
  frame->mutable_open_space_info()->
      mutable_path_planning_trajectory_result()->clear();
  frame->mutable_open_space_info()->
      mutable_interpolated_trajectory_result()->clear();
  frame->mutable_open_space_info()->
      mutable_partitioned_trajectories()->clear();
  frame->mutable_open_space_info()->
      mutable_chosen_partitioned_trajectory()->first.clear();
  return StageResult(StageStatusType::FINISHED);
  // const auto vehicle_status = injector_->vehicle_state();
  // if (std::fabs(vehicle_status->steering_percentage()) <
  //     GetContextAs<ParkAndGoContext>()
  //         ->scenario_config.max_steering_percentage_when_cruise()) {
  //   next_stage_ = "VALET_PARKING_RETRY";
  //   return StageResult(StageStatusType::FINISHED);
  // }
  // return StageResult(StageStatusType::RUNNING);
}

bool StageParkingPark::CheckADCParkingCompleted(Frame* frame) {
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
  if (frame->open_space_info().parking_type() == ParkingType::PARALLEL_PARKING &&
      !scenario_config_.enable_parallel_retry_parking()){
    AINFO << "parallel parking no retry";
    return true;
  }
  if (std::fabs(lat_error) < scenario_config_.max_lat_error() &&
      std::fabs(lon_error) < scenario_config_.max_lon_error() &&
      std::fabs(delta_theta) < scenario_config_.max_heading_error()) {
    return true;
  }
  return false;
}

bool StageParkingPark::CheckParkingMissionInfo(const ValetParkingContext& planning_command) {
  if (parking_mission_info_.parking_spot_id != planning_command.target_parking_spot_id) {
    parking_mission_info_.parking_spot_id = planning_command.target_parking_spot_id;
    parking_mission_info_.command_sequence_num = planning_command.command_sequence_num;
    parking_mission_info_.status = ParkingMissionStatus::RUNNING;
    AINFO << "Get new parking spot id from routing: " << parking_mission_info_.parking_spot_id
          << " command sequence num: " << (int)parking_mission_info_.command_sequence_num;
    return true;
  } else if (parking_mission_info_.command_sequence_num != planning_command.command_sequence_num) {
    parking_mission_info_.command_sequence_num = planning_command.command_sequence_num;
    parking_mission_info_.status = ParkingMissionStatus::RUNNING;
    AINFO << "Get new command sequence num from routing: " << parking_mission_info_.parking_spot_id
          << " command sequence num: " << (int)parking_mission_info_.command_sequence_num;
    return true;
  } else if (parking_mission_info_.status == ParkingMissionStatus::RUNNING) {
    AINFO << "Get same parking spot id from routing: " << parking_mission_info_.parking_spot_id
          << " command sequence num: " << (int)parking_mission_info_.command_sequence_num;
    return true;
  } else if (parking_mission_info_.status == ParkingMissionStatus::DONE) {
    AINFO << "Get same parking spot id from routing: " << parking_mission_info_.parking_spot_id << "but status is DONE";
    return false;
  }
  AINFO << "UNKNOW STATUS";
  return false;
}

}  // namespace planning
}  // namespace apollo
