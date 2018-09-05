/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_base.h"

#include <algorithm>
#include <list>
#include <vector>

#include "google/protobuf/repeated_field.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/open_space/open_space_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"
#include "modules/planning/toolkits/deciders/traffic_decider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;

PlanningBase::~PlanningBase() {}

void PlanningBase::CheckPlanningConfig() {
  if (config_.has_lane_follow_scenario_config() &&
      config_.lane_follow_scenario_config().has_dp_st_speed_config()) {
    const auto& dp_st_speed_config =
        config_.lane_follow_scenario_config().dp_st_speed_config();
    CHECK(dp_st_speed_config.has_matrix_dimension_s());
    CHECK_GT(dp_st_speed_config.matrix_dimension_s(), 3);
    CHECK_LT(dp_st_speed_config.matrix_dimension_s(), 10000);
    CHECK(dp_st_speed_config.has_matrix_dimension_t());
    CHECK_GT(dp_st_speed_config.matrix_dimension_t(), 3);
    CHECK_LT(dp_st_speed_config.matrix_dimension_t(), 10000);
  }
  // TODO(All): check other config params
}

bool PlanningBase::IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

void PlanningBase::PublishPlanningPb(ADCTrajectory* trajectory_pb,
                                     double timestamp) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (AdapterManager::GetPrediction() &&
      !AdapterManager::GetPrediction()->Empty()) {
    const auto& prediction =
        AdapterManager::GetPrediction()->GetLatestObserved();
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        prediction.header().lidar_timestamp());
    trajectory_pb->mutable_header()->set_camera_timestamp(
        prediction.header().camera_timestamp());
    trajectory_pb->mutable_header()->set_radar_timestamp(
        prediction.header().radar_timestamp());
  }

  // TODO(all): integrate reverse gear
  trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
  if (AdapterManager::GetRoutingResponse() &&
      !AdapterManager::GetRoutingResponse()->Empty()) {
    trajectory_pb->mutable_routing_header()->CopyFrom(
        AdapterManager::GetRoutingResponse()->GetLatestObserved().header());
  }

  if (FLAGS_use_planning_fallback &&
      trajectory_pb->trajectory_point_size() == 0) {
    SetFallbackTrajectory(trajectory_pb);
  }

  // NOTICE:
  // Since we are using the time at each cycle beginning as timestamp, the
  // relative time of each trajectory point should be modified so that we can
  // use the current timestamp in header.

  // auto* trajectory_points = trajectory_pb.mutable_trajectory_point();
  if (!FLAGS_planning_test_mode) {
    const double dt = timestamp - Clock::NowInSeconds();
    for (auto& p : *trajectory_pb->mutable_trajectory_point()) {
      p.set_relative_time(p.relative_time() + dt);
    }
  }
  Publish(trajectory_pb);
}

void PlanningBase::SetFallbackTrajectory(ADCTrajectory* trajectory_pb) {
  CHECK_NOTNULL(trajectory_pb);
  // use planning trajecotry from last cycle
  auto* last_planning = AdapterManager::GetPlanning();
  if (last_planning != nullptr && !last_planning->Empty()) {
    const auto& traj = last_planning->GetLatestObserved();

    const double current_time_stamp = trajectory_pb->header().timestamp_sec();
    const double pre_time_stamp = traj.header().timestamp_sec();

    for (int i = 0; i < traj.trajectory_point_size(); ++i) {
      const double t = traj.trajectory_point(i).relative_time() +
                       pre_time_stamp - current_time_stamp;
      auto* p = trajectory_pb->add_trajectory_point();
      p->CopyFrom(traj.trajectory_point(i));
      p->set_relative_time(t);
    }
  }
}

}  // namespace planning
}  // namespace apollo
