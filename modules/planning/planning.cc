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

#include "modules/planning/planning.h"

#include <algorithm>
#include <google/protobuf/repeated_field.h>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/data_center.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planner/em/em_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::common::Status;
using apollo::common::ErrorCode;

std::string Planning::Name() const { return "planning"; }

void Planning::RegisterPlanners() {
  planner_factory_.Register(
      PlanningConfig::RTK, []() -> Planner* { return new RTKReplayPlanner(); });
  planner_factory_.Register(PlanningConfig::EM,
                            []() -> Planner* { return new EMPlanner(); });
}

Status Planning::Init() {
  if (!apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file,
                                              &config_)) {
    AERROR << "failed to load planning config file "
           << FLAGS_planning_config_file;
    return Status(
        ErrorCode::PLANNING_ERROR,
        "failed to load planning config file: " + FLAGS_planning_config_file);
  }

  AdapterManager::Init(FLAGS_adapter_config_path);
  if (AdapterManager::GetLocalization() == nullptr) {
    std::string error_msg("Localization is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (AdapterManager::GetChassis() == nullptr) {
    std::string error_msg("chassis is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  if (AdapterManager::GetRoutingResult() == nullptr) {
    std::string error_msg("RoutingResult is not registered");
    AERROR << error_msg;
    return Status(ErrorCode::PLANNING_ERROR, error_msg);
  }
  // TODO(all) temporarily use the offline routing data.
  if (!AdapterManager::GetRoutingResult()->HasReceived()) {
    if (!AdapterManager::GetRoutingResult()->FeedFile(
            FLAGS_offline_routing_file)) {
      auto error_msg = common::util::StrCat(
          "Failed to load offline routing file ", FLAGS_offline_routing_file);
      AERROR << error_msg;
      return Status(ErrorCode::PLANNING_ERROR, error_msg);
    } else {
      AWARN << "Using offline prouting file " << FLAGS_offline_routing_file;
    }
  }
  RegisterPlanners();
  planner_ = planner_factory_.CreateObject(config_.planner_type());
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  return planner_->Init(config_);
}

Status Planning::Start() {
  static ros::Rate loop_rate(FLAGS_planning_loop_rate);
  while (ros::ok()) {
    RunOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return Status::OK();
}

void Planning::RunOnce() {
  AdapterManager::Observe();
  if (AdapterManager::GetLocalization()->Empty()) {
    AERROR << "Localization is not available; skip the planning cycle";
    return;
  }

  if (AdapterManager::GetChassis()->Empty()) {
    AERROR << "Chassis is not available; skip the planning cycle";
    return;
  }
  if (AdapterManager::GetRoutingResult()->Empty()) {
    AERROR << "RoutingResult is not available; skip the planning cycle";
    return;
  }

  AINFO << "Start planning ...";

  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  ADEBUG << "Get localization:" << localization.DebugString();

  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  ADEBUG << "Get chassis:" << chassis.DebugString();

  common::VehicleState::instance()->Update(localization, chassis);

  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  // the execution_start_time is the estimated time when the planned trajectory
  // will be executed by the controller.
  double execution_start_time =
      apollo::common::time::ToSecond(apollo::common::time::Clock::Now()) +
      planning_cycle_time;

  if (!DataCenter::instance()->init_frame(
          AdapterManager::GetPlanning()->GetSeqNum() + 1)) {
    AERROR << "DataCenter init frame failed";
    return;
  }

  ADCTrajectory trajectory_pb;
  bool is_auto_mode = chassis.driving_mode() == chassis.COMPLETE_AUTO_DRIVE;
  bool res_planning = Plan(is_auto_mode, execution_start_time, &trajectory_pb);
  if (res_planning) {
    AdapterManager::FillPlanningHeader("planning",
                                       trajectory_pb.mutable_header());
    trajectory_pb.mutable_header()->set_timestamp_sec(execution_start_time);
    AdapterManager::PublishPlanning(trajectory_pb);
    ADEBUG << "Planning succeeded:" << trajectory_pb.header().DebugString();
  } else {
    AERROR << "Planning failed";
  }
}

void Planning::Stop() {}

bool Planning::Plan(const bool is_on_auto_mode, const double publish_time,
                    ADCTrajectory* trajectory_pb) {
  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  double execution_start_time = publish_time;

  if (is_on_auto_mode && !last_trajectory_.empty()) {
    // if the auto-driving mode is on and we have the trajectory from last
    // cycle, then
    // find the planning starting point from the last planning result.
    // this ensures the smoothness of planning output and
    // therefore the smoothness of control execution.

    auto matched_info =
        ComputeStartingPointFromLastTrajectory(execution_start_time);
    TrajectoryPoint matched_point = matched_info.first;
    std::uint32_t matched_index = matched_info.second;

    // Compute the position deviation between current vehicle
    // position and target vehicle position.
    // If the deviation exceeds a specific threshold,
    // it will be unsafe to planning from the matched point.
    double dx =
        matched_point.path_point().x() - common::VehicleState::instance()->x();
    double dy =
        matched_point.path_point().y() - common::VehicleState::instance()->y();
    double position_deviation = std::sqrt(dx * dx + dy * dy);

    if (position_deviation < FLAGS_replanning_threshold) {
      // planned trajectory from the matched point, the matched point has
      // relative time 0.
      auto status = planner_->MakePlan(matched_point, trajectory_pb);

      if (!status.ok()) {
        last_trajectory_.clear();
        return false;
      }

      // a segment of last trajectory to be attached to planned trajectory in
      // case controller needs.
      GetOverheadTrajectory(matched_index, FLAGS_rtk_trajectory_backward,
                            trajectory_pb);

      // store the planned trajectory and header info for next planning cycle.
      last_trajectory_ = {trajectory_pb->trajectory_point().begin(),
                          trajectory_pb->trajectory_point().end()};
      last_header_time_ = execution_start_time;
      return true;
    }
  }

  // if 1. the auto-driving mode is off or
  //    2. we don't have the trajectory from last planning cycle or
  //    3. the position deviation from actual and target is too high
  // then planning from current vehicle state.
  TrajectoryPoint vehicle_state_point =
      ComputeStartingPointFromVehicleState(planning_cycle_time);

  auto status = planner_->MakePlan(vehicle_state_point, trajectory_pb);
  if (!status.ok()) {
    last_trajectory_.clear();
    return false;
  }
  // store the planned trajectory and header info for next planning cycle.
  last_trajectory_ = {trajectory_pb->trajectory_point().begin(),
                      trajectory_pb->trajectory_point().end()};
  last_header_time_ = execution_start_time;
  return true;
}

std::pair<TrajectoryPoint, std::uint32_t>
Planning::ComputeStartingPointFromLastTrajectory(
    const double start_time) const {
  auto comp = [](const TrajectoryPoint& p, const double t) {
    return p.relative_time() < t;
  };

  auto it_lower =
      std::lower_bound(last_trajectory_.begin(), last_trajectory_.end(),
                       start_time - last_header_time_, comp);
  if (it_lower == last_trajectory_.end()) {
    it_lower--;
  }
  std::uint32_t index = it_lower - last_trajectory_.begin();
  return std::pair<TrajectoryPoint, std::uint32_t>(*it_lower, index);
}

TrajectoryPoint Planning::ComputeStartingPointFromVehicleState(
    const double forward_time) const {
  // common::math::Vec2d estimated_position =
  // VehicleState::instance()->EstimateFuturePosition(forward_time);
  TrajectoryPoint point;
  // point.set_x(estimated_position.x());
  // point.set_y(estimated_position.y());
  point.mutable_path_point()->set_x(common::VehicleState::instance()->x());
  point.mutable_path_point()->set_y(common::VehicleState::instance()->y());
  point.mutable_path_point()->set_z(common::VehicleState::instance()->z());
  point.set_v(common::VehicleState::instance()->linear_velocity());
  point.set_a(common::VehicleState::instance()->linear_acceleration());
  point.mutable_path_point()->set_theta(common::VehicleState::instance()->heading());  
  point.mutable_path_point()->set_kappa(0.0);
  const double speed_threshold = 0.1;
  if (point.v() > speed_threshold) {
    point.mutable_path_point()->set_kappa(
        common::VehicleState::instance()->angular_velocity() /
        common::VehicleState::instance()->linear_velocity());
  }
  point.mutable_path_point()->set_dkappa(0.0);
  point.mutable_path_point()->set_s(0.0);
  point.set_relative_time(0.0);
  return point;
}

void Planning::Reset() {
  last_header_time_ = 0.0;
  last_trajectory_.clear();
}

void Planning::GetOverheadTrajectory(const std::uint32_t matched_index,
                                     const std::uint32_t buffer_size,
                                     ADCTrajectory* trajectory_pb) {
  const std::uint32_t start_index =
      matched_index < buffer_size ? 0 : matched_index - buffer_size;

  google::protobuf::RepeatedPtrField<TrajectoryPoint> overhead_trajectory(
      last_trajectory_.begin() + start_index,
      last_trajectory_.begin() + matched_index);

  double zero_relative_time = last_trajectory_[matched_index].relative_time();
  // reset relative time
  for (auto& p : overhead_trajectory) {
    p.set_relative_time(p.relative_time() - zero_relative_time);
  }

  // Insert the overhead_trajectory to the head of trajectory_pb.
  overhead_trajectory.MergeFrom(trajectory_pb->trajectory_point());
  trajectory_pb->mutable_trajectory_point()->Swap(&overhead_trajectory);
}

}  // namespace planning
}  // namespace apollo
