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

/**
 * @file speed_optimizer.cc
 **/

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/tasks/speed_optimizer.h"

namespace apollo {
namespace planning {

SpeedOptimizer::SpeedOptimizer(const std::string& name) : Task(name) {}

apollo::common::Status SpeedOptimizer::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);

  auto ret = Process(
      reference_line_info->AdcSlBoundary(), reference_line_info->path_data(),
      frame->PlanningStartPoint(), reference_line_info->reference_line(),
      reference_line_info->path_decision(),
      reference_line_info->mutable_speed_data());
  RecordDebugInfo(reference_line_info->speed_data());
  return ret;
}

void SpeedOptimizer::RecordDebugInfo(const SpeedData& speed_data) {
  auto debug = frame_->MutableADCTrajectory()->mutable_debug();
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
  ptr_speed_plan->set_name(Name());
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
}

void SpeedOptimizer::RecordSTGraphDebug(
    const std::vector<StBoundary>& boundaries, const SpeedLimit& speed_limits,
    const SpeedData& speed_data) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  auto debug = frame_->MutableADCTrajectory()->mutable_debug();
  auto st_graph_debug = debug->mutable_planning_data()->add_st_graph();
  st_graph_debug->set_name(Name());
  for (const auto& boundary : boundaries) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    switch (boundary.boundary_type()) {
      case StBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(
            ::apollo::planning_internal::StGraphBoundaryDebug::
                ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case StBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            ::apollo::planning_internal::StGraphBoundaryDebug::
                ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case StBoundary::BoundaryType::STOP:
        boundary_debug->set_type(
            ::apollo::planning_internal::StGraphBoundaryDebug::
                ST_BOUNDARY_TYPE_STOP);
        break;
      case StBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            ::apollo::planning_internal::StGraphBoundaryDebug::
                ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case StBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(
            ::apollo::planning_internal::StGraphBoundaryDebug::
                ST_BOUNDARY_TYPE_YIELD);
        break;
    }

    for (const auto& point : boundary.points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto& point : speed_limits.speed_limit_points()) {
    common::SpeedPoint speed_point;
    speed_point.set_s(point.first);
    speed_point.set_v(point.second);
    st_graph_debug->add_speed_limit()->CopyFrom(speed_point);
  }

  st_graph_debug->mutable_speed_profile()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
}

}  // namespace planning
}  // namespace apollo
