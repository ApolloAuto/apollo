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

bool PlanningBase::IsPlanningFinished(
    const ADCTrajectory::TrajectoryType& current_trajectory_type) const {
  const auto frame = injector_->frame_history()->Latest();
  if (current_trajectory_type == apollo::planning::ADCTrajectory::OPEN_SPACE) {
    AINFO << "Current trajectory type is: OPEN SPACE";
    if (frame->open_space_info().openspace_planning_finish()) {
      AINFO << "OPEN SPACE: planning finished";
      return true;
    } else {
      AINFO << "OPEN SPACE: planning not finished";
      return false;
    }
  } else {
    // const auto frame = injector_->frame_history()->Latest();
    if (nullptr == frame || frame->reference_line_info().empty() ||
        nullptr == local_view_.planning_command) {
      AINFO << "Current reference point is empty;";
      return true;
    }
    const auto& reference_line_info = frame->reference_line_info().front();
    // Check if the ReferenceLineInfo is the last passage.
    const auto& reference_points =
        reference_line_info.reference_line().reference_points();
    if (reference_points.empty()) {
      AINFO << "Current reference points is empty;";
      return true;
    }
    const auto& last_reference_point = reference_points.back();
    const std::vector<hdmap::LaneWaypoint>& lane_way_points =
        last_reference_point.lane_waypoints();
    if (lane_way_points.empty()) {
      AINFO << "Last reference point is empty;";
      return true;
    }
    // Get the end lane way point.
    if (nullptr == frame->local_view().end_lane_way_point) {
      AINFO << "Current end lane way is empty;";
      return true;
    }
    bool is_has_passed_destination = injector_->planning_context()
                                         ->planning_status()
                                         .destination()
                                         .has_passed_destination();
    AINFO << "Current passed destination:" << is_has_passed_destination;
    return is_has_passed_destination;
  }
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

bool PlanningBase::GenerateWidthOfLane(const Vec2d& current_location,
                                       Vec2d& left_point, Vec2d& right_point) {
  double left_width = 0, right_width = 0;
  const auto frame = injector_->frame_history()->Latest();
  if (nullptr == frame || frame->reference_line_info().empty()) {
    AINFO << "Reference lane is empty!";
    return false;
  }
  const auto& reference_line_info = frame->reference_line_info().front();
  // get current SL
  common::SLPoint current_sl;
  reference_line_info.reference_line().XYToSL(current_location, &current_sl);
  // Get the lane width of vehicle location
  bool get_width_of_lane = reference_line_info.reference_line().GetLaneWidth(
      current_sl.s(), &left_width, &right_width);
  AINFO << "get_width_of_lane: " << get_width_of_lane
        << ", left_width: " << left_width << ", right_width: " << right_width;
  if (get_width_of_lane && left_width != 0 && right_width != 0) {
    AINFO << "Get the width of lane successfully!";
    SLPoint sl_left_point, sl_right_point;
    sl_left_point.set_s(current_sl.s());
    sl_left_point.set_l(left_width);
    sl_right_point.set_s(current_sl.s());
    sl_right_point.set_l(-right_width);
    reference_line_info.reference_line().SLToXY(sl_left_point, &left_point);
    reference_line_info.reference_line().SLToXY(sl_right_point, &right_point);
    return true;
  } else {
    AINFO << "Failed to get the width of lane!";
    return false;
  }
}

}  // namespace planning
}  // namespace apollo
