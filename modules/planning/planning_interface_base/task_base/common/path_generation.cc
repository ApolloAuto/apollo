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

/**
 * @file
 **/
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

apollo::common::Status PathGeneration::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(frame, reference_line_info);
}

apollo::common::Status PathGeneration::Execute(Frame* frame) {
  Task::Execute(frame);
  return Process(frame);
}

void PathGeneration::RecordDebugInfo(
    const PathBound& path_boundaries, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  if (path_boundaries.empty()) {
    AINFO << "path boundary is empty!";
    return;
  }
  CHECK_NOTNULL(reference_line_info);

  // Take the left and right path boundaries, and transform them into two
  // PathData so that they can be displayed in simulator.
  std::vector<common::FrenetFramePoint> frenet_frame_left_boundaries;
  std::vector<common::FrenetFramePoint> frenet_frame_right_boundaries;
  for (const PathBoundPoint& path_bound_point : path_boundaries) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(path_bound_point.s);
    frenet_frame_point.set_dl(0.0);
    frenet_frame_point.set_ddl(0.0);

    frenet_frame_point.set_l(path_bound_point.l_lower.l);
    frenet_frame_right_boundaries.push_back(frenet_frame_point);
    frenet_frame_point.set_l(path_bound_point.l_upper.l);
    frenet_frame_left_boundaries.push_back(frenet_frame_point);
  }

  auto frenet_frame_left_path =
      FrenetFramePath(std::move(frenet_frame_left_boundaries));
  auto frenet_frame_right_path =
      FrenetFramePath(std::move(frenet_frame_right_boundaries));

  PathData left_path_data;
  left_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  left_path_data.SetFrenetPath(std::move(frenet_frame_left_path));
  PathData right_path_data;
  right_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  right_path_data.SetFrenetPath(std::move(frenet_frame_right_path));

  // Insert the transformed PathData into the simulator display.
  auto* ptr_display_path_1 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_1->set_name(std::string("planning_path_boundary_1_") +
                               debug_name);
  ptr_display_path_1->mutable_path_point()->CopyFrom(
      {left_path_data.discretized_path().begin(),
       left_path_data.discretized_path().end()});
  auto* ptr_display_path_2 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_2->set_name(std::string("planning_path_boundary_2_") +
                               debug_name);
  ptr_display_path_2->mutable_path_point()->CopyFrom(
      {right_path_data.discretized_path().begin(),
       right_path_data.discretized_path().end()});
}

void PathGeneration::RecordDebugInfo(
    const PathData& path_data, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  const auto& path_points = path_data.discretized_path();
  auto* ptr_optimized_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_optimized_path->set_name(std::string("candidate_path_") + debug_name);
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}
void PathGeneration::GetStartPointSLState() {
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  common::TrajectoryPoint planning_start_point = frame_->PlanningStartPoint();
  if (FLAGS_use_front_axe_center_in_path_planning) {
    double front_to_rear_axe_distance =
        apollo::common::VehicleConfigHelper::GetConfig()
            .vehicle_param()
            .wheel_base();
    planning_start_point.mutable_path_point()->set_x(
        planning_start_point.path_point().x() +
        front_to_rear_axe_distance *
            std::cos(planning_start_point.path_point().theta()));
    planning_start_point.mutable_path_point()->set_y(
        planning_start_point.path_point().y() +
        front_to_rear_axe_distance *
            std::sin(planning_start_point.path_point().theta()));
  }
  AINFO << std::fixed << "Plan at the starting point: x = "
        << planning_start_point.path_point().x()
        << ", y = " << planning_start_point.path_point().y()
        << ", and angle = " << planning_start_point.path_point().theta();

  // Initialize some private variables.
  // ADC s/l info.
  init_sl_state_ = reference_line.ToFrenetFrame(planning_start_point);
}

bool PathGeneration::GetSLBoundary(const PathData& path_data, int point_index,
                                   const ReferenceLineInfo* reference_line_info,
                                   SLBoundary* const sl_boundary) {
  CHECK_NOTNULL(sl_boundary);
  const auto& discrete_path = path_data.discretized_path();
  if (point_index < 0 ||
      static_cast<size_t>(point_index) > discrete_path.size()) {
    return false;
  }
  sl_boundary->mutable_boundary_point()->Clear();
  // Get vehicle config parameters.
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double ego_length = vehicle_config.vehicle_param().length();
  const double ego_width = vehicle_config.vehicle_param().width();
  const double ego_back_to_center =
      vehicle_config.vehicle_param().back_edge_to_center();
  const double ego_center_shift_distance =
      ego_length / 2.0 - ego_back_to_center;
  // Generate vehicle bounding box.
  const auto& rear_center_path_point = discrete_path[point_index];
  const double ego_theta = rear_center_path_point.theta();
  common::math::Box2d ego_box(
      {rear_center_path_point.x(), rear_center_path_point.y()}, ego_theta,
      ego_length, ego_width);
  common::math::Vec2d shift_vec{
      ego_center_shift_distance * ego_box.cos_heading(),
      ego_center_shift_distance * ego_box.sin_heading()};
  ego_box.Shift(shift_vec);
  // Set warm_start_s near the geometry center of the box.
  double warm_start_s = path_data.frenet_frame_path()[point_index].s() +
                        ego_center_shift_distance;
  // Get the SL boundary of vehicle.
  reference_line_info->reference_line().GetSLBoundary(ego_box, sl_boundary,
                                                      warm_start_s);
  return true;
}

}  // namespace planning
}  // namespace apollo
