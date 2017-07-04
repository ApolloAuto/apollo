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

#include "modules/planning/planning_node.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

using apollo::common::vehicle_state::VehicleState;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using TrajectoryPb = ADCTrajectory;

PlanningNode::PlanningNode() {
  AdapterManager::Init(FLAGS_adapter_config_path);
}

PlanningNode::~PlanningNode() {}

void PlanningNode::Run() {
  static ros::Rate loop_rate(FLAGS_planning_loop_rate);
  while (ros::ok()) {
    RunOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void PlanningNode::RunOnce() {
  AdapterManager::Observe();
  if (AdapterManager::GetLocalization() == nullptr) {
    AERROR << "Localization is not available; skip the planning cycle";
    return;
  }
  if (AdapterManager::GetLocalization()->Empty()) {
    AERROR << "localization messages are missing; skip the planning cycle";
    return;
  } else {
    AINFO << "Get localization message;";
  }

  if (AdapterManager::GetChassis() == nullptr) {
    AERROR << "Chassis is not available; skip the planning cycle";
    return;
  }
  if (AdapterManager::GetChassis()->Empty()) {
    AERROR << "Chassis messages are missing; skip the planning cycle";
    return;
  } else {
    AINFO << "Get localization message;";
  }

  AINFO << "Start planning ...";

  const auto& localization =
      AdapterManager::GetLocalization()->GetLatestObserved();
  VehicleState vehicle_state(localization);

  const auto& chassis = AdapterManager::GetChassis()->GetLatestObserved();
  bool is_on_auto_mode = chassis.driving_mode() == chassis.COMPLETE_AUTO_DRIVE;

  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  // the execution_start_time is the estimated time when the planned trajectory
  // will be executed by the controller.
  double execution_start_time =
      apollo::common::time::ToSecond(apollo::common::time::Clock::Now()) +
      planning_cycle_time;

  std::vector<TrajectoryPoint> planning_trajectory;
  bool res_planning = planning_.Plan(vehicle_state, is_on_auto_mode,
      execution_start_time, &planning_trajectory);
  if (res_planning) {
    TrajectoryPb trajectory_pb = ToTrajectoryPb(execution_start_time, planning_trajectory);
    AdapterManager::PublishPlanningTrajectory(trajectory_pb);
    AINFO << "Planning succeeded";
  } else {
    AINFO << "Planning failed";
  }
}

void PlanningNode::Reset() {
  planning_.Reset();
}

TrajectoryPb PlanningNode::ToTrajectoryPb(
    const double header_time,
    const std::vector<TrajectoryPoint>& discretized_trajectory) {
  TrajectoryPb trajectory_pb;
  AdapterManager::FillPlanningTrajectoryHeader("planning",
                                               trajectory_pb.mutable_header());

  trajectory_pb.mutable_header()->set_timestamp_sec(header_time);

  for (const auto& trajectory_point : discretized_trajectory) {
    auto ptr_trajectory_point_pb = trajectory_pb.add_adc_trajectory_point();
    ptr_trajectory_point_pb->set_x(trajectory_point.x);
    ptr_trajectory_point_pb->set_y(trajectory_point.y);
    ptr_trajectory_point_pb->set_theta(trajectory_point.theta);
    ptr_trajectory_point_pb->set_curvature(trajectory_point.kappa);
    ptr_trajectory_point_pb->set_relative_time(trajectory_point.relative_time);
    ptr_trajectory_point_pb->set_speed(trajectory_point.v);
    ptr_trajectory_point_pb->set_acceleration_s(trajectory_point.a);
    ptr_trajectory_point_pb->set_accumulated_s(trajectory_point.s);
  }
  return std::move(trajectory_pb);
}

}  // namespace planning
}  // namespace apollo
