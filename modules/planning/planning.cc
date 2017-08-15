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
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planner/rtk_replay_planner.h"
#include "modules/planning/planning.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterManager;
using TrajectoryPb = ADCTrajectory;

void Planning::RegisterPlanners() {
  planner_factory_.Register(PlannerType::RTK_PLANNER, []() -> Planner * {
    return new RTKReplayPlanner();
  });
}

Planning::Planning() { RegisterPlanners(); }

bool Planning::Plan(const common::vehicle_state::VehicleState &vehicle_state,
                    const bool is_on_auto_mode, const double publish_time,
                    std::vector<TrajectoryPoint> *planning_trajectory) {
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  double execution_start_time = publish_time;

  ptr_planner_ = planner_factory_.CreateObject(PlannerType::RTK_PLANNER);
  if (!ptr_planner_) {
    AERROR << "Failed to create RTK planner";
    return false;
  }

  if (is_on_auto_mode && !last_trajectory_.empty()) {
    // if the auto-driving mode is on and we have the trajectory from last
    // cycle, then
    // find the planning starting point from the last planning result.
    // this ensures the smoothness of planning output and
    // therefore the smoothness of control execution.

    auto matched_info =
        ComputeStartingPointFromLastTrajectory(execution_start_time);
    TrajectoryPoint matched_point = matched_info.first;
    std::size_t matched_index = matched_info.second;

    // Compute the position deviation between current vehicle
    // position and target vehicle position.
    // If the deviation exceeds a specific threshold,
    // it will be unsafe to planning from the matched point.
    double dx = matched_point.x() - vehicle_state.x();
    double dy = matched_point.y() - vehicle_state.y();
    double position_deviation = std::sqrt(dx * dx + dy * dy);

    if (position_deviation < FLAGS_replanning_threshold) {
      // planned trajectory from the matched point, the matched point has
      // relative time 0.
      bool planning_succeeded =
          ptr_planner_->Plan(matched_point, planning_trajectory);

      if (!planning_succeeded) {
        last_trajectory_.clear();
        return false;
      }

      // a segment of last trajectory to be attached to planned trajectory in
      // case controller needs.
      auto overhead_trajectory = GetOverheadTrajectory(
          matched_index, (std::size_t)FLAGS_rtk_trajectory_backward);
      planning_trajectory->insert(planning_trajectory->begin(),
                                  overhead_trajectory.begin(),
                                  overhead_trajectory.end());

      // store the planned trajectory and header info for next planning cycle.
      last_trajectory_ = *planning_trajectory;
      last_header_time_ = execution_start_time;
      return true;
    }
  }

  // if 1. the auto-driving mode is off or
  //    2. we don't have the trajectory from last planning cycle or
  //    3. the position deviation from actual and target is too high
  // then planning from current vehicle state.
  TrajectoryPoint vehicle_state_point =
      ComputeStartingPointFromVehicleState(vehicle_state, planning_cycle_time);

  bool planning_succeeded =
      ptr_planner_->Plan(vehicle_state_point, planning_trajectory);
  if (!planning_succeeded) {
    last_trajectory_.clear();
    return false;
  }
  // store the planned trajectory and header info for next planning cycle.
  last_trajectory_ = *planning_trajectory;
  last_header_time_ = execution_start_time;
  return true;
}

std::pair<TrajectoryPoint, std::size_t>
Planning::ComputeStartingPointFromLastTrajectory(
    const double start_time) const {
  auto comp = [](const TrajectoryPoint &p, const double t) {
    return p.relative_time() < t;
  };

  auto it_lower =
      std::lower_bound(last_trajectory_.begin(), last_trajectory_.end(),
                       start_time - last_header_time_, comp);
  if (it_lower == last_trajectory_.end()) {
    it_lower--;
  }
  std::size_t index = it_lower - last_trajectory_.begin();
  return std::pair<TrajectoryPoint, std::size_t>(*it_lower, index);
}

TrajectoryPoint Planning::ComputeStartingPointFromVehicleState(
    const common::vehicle_state::VehicleState &vehicle_state,
    const double forward_time) const {
  // Eigen::Vector2d estimated_position =
  // vehicle_state.EstimateFuturePosition(forward_time);
  TrajectoryPoint point;
  // point.set_x(estimated_position.x());
  // point.set_y(estimated_position.y());
  point.set_x(vehicle_state.x());
  point.set_y(vehicle_state.y());
  point.set_z(vehicle_state.z());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_kappa(0.0);
  const double speed_threshold = 0.1;
  if (point.v() > speed_threshold) {
    point.set_kappa(vehicle_state.angular_velocity() /
                    vehicle_state.linear_velocity());
  }
  point.set_dkappa(0.0);
  point.set_s(0.0);
  point.set_relative_time(0.0);
  return point;
}

void Planning::Reset() {
  last_header_time_ = 0.0;
  last_trajectory_.clear();
}

std::vector<TrajectoryPoint> Planning::GetOverheadTrajectory(
    const std::size_t matched_index, const std::size_t buffer_size) {
  const std::size_t start_index =
      matched_index < buffer_size ? 0 : matched_index - buffer_size;

  std::vector<TrajectoryPoint> overhead_trajectory(
      last_trajectory_.begin() + start_index,
      last_trajectory_.begin() + matched_index);

  double zero_relative_time = last_trajectory_[matched_index].relative_time();
  // reset relative time
  for (auto &p : overhead_trajectory) {
    p.set_relative_time(p.relative_time() - zero_relative_time);
  }
  return overhead_trajectory;
}

}  // namespace planning
}  // namespace apollo
