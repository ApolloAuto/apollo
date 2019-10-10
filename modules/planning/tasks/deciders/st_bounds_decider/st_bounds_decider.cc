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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"

#include <vector>

#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

STBoundsDecider::STBoundsDecider(const TaskConfig& config)
    : Decider(config) {
  CHECK(config.has_st_bounds_decider_config());
  st_bounds_config_ = config.st_bounds_decider_config();
}

Status STBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  const PathData &path_data = reference_line_info->path_data();
  PathDecision *const path_decision = reference_line_info->path_decision();

  // Map all related obstacles onto ST-Graph.
  auto time1 = std::chrono::system_clock::now();
  STObstaclesProcessor st_obstacles_processor(
      path_data.discretized_path().Length(), st_bounds_config_.total_time(),
      path_data);
  st_obstacles_processor.MapObstaclesToSTBoundaries(path_decision);
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff.count() * 1000
         << " msec.";
  // Record the ST-Graph for good visualization and easy debugging.
  auto all_st_boundaries = st_obstacles_processor.GetAllSTBoundaries();
  std::vector<STBoundary> st_boundaries;
  for (auto it = all_st_boundaries.begin();
       it != all_st_boundaries.end(); ++it) {
    st_boundaries.push_back(it->second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug *st_graph_debug = reference_line_info_->mutable_debug()->
      mutable_planning_data()->add_st_graph();
  RecordSTGraphDebug(st_boundaries, st_graph_debug);

  // Initialize Guide-Line and Driving-Limits.
  // TODO(jiacheng): implement this.

  // Sweep the t-axis, and determine the s-boundaries step by step.
  // TODO(jiacheng): implement this.

  return Status::OK();
}

void STBoundsDecider::RecordSTGraphDebug(
    const std::vector<STBoundary>& st_graph_data,
    planning_internal::STGraphDebug* const st_graph_debug) {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  for (const auto& boundary : st_graph_data) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    ADEBUG << "Obstacle ID = " << boundary.id();
    boundary_debug->set_type(
        StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);

    for (const auto &point : boundary.points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }
}

}  // namespace planning
}  // namespace apollo
