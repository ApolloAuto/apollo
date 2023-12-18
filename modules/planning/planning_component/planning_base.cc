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

#include "modules/planning/planning_component/planning_base.h"

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/time/clock.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/local_view.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planning_base/common/util/config_util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_interface_base/planner_base/planner.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

PlanningBase::PlanningBase(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

PlanningBase::~PlanningBase() {}

Status PlanningBase::Init(const PlanningConfig& config) {
  injector_->planning_context()->Init();
  config_ = config;
  return Status::OK();
}

bool PlanningBase::IsPlanningFinished() const {
  const auto frame = injector_->frame_history()->Latest();
  if (nullptr == frame || frame->reference_line_info().empty() ||
      nullptr == local_view_.planning_command) {
    return true;
  }
  const auto& reference_line_info = frame->reference_line_info().front();
  // Check if the ReferenceLineInfo is the last passage.
  const auto& reference_points =
      reference_line_info.reference_line().reference_points();
  if (reference_points.empty()) {
    return true;
  }
  const auto& last_reference_point = reference_points.back();
  const std::vector<hdmap::LaneWaypoint>& lane_way_points =
      last_reference_point.lane_waypoints();
  if (lane_way_points.empty()) {
    return true;
  }
  // Get the end lane way point.
  if (nullptr == frame->local_view().end_lane_way_point) {
    return true;
  }
  return injector_->planning_context()
      ->planning_status()
      .destination()
      .has_passed_destination();
}

void PlanningBase::FillPlanningPb(const double timestamp,
                                  ADCTrajectory* const trajectory_pb) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (local_view_.prediction_obstacles->has_header()) {
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());
    trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());
    trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }
  trajectory_pb->mutable_routing_header()->CopyFrom(
      local_view_.planning_command->header());
}

void PlanningBase::LoadPlanner() {
  // Use PublicRoadPlanner as default Planner
  std::string planner_name = "apollo::planning::PublicRoadPlanner";
  if ("" != config_.planner()) {
    planner_name = config_.planner();
    planner_name = ConfigUtil::GetFullPlanningClassName(planner_name);
  }
  planner_ =
      cyber::plugin_manager::PluginManager::Instance()->CreateInstance<Planner>(
          planner_name);
}

}  // namespace planning
}  // namespace apollo
