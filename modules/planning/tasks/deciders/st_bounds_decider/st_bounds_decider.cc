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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

STBoundsDecider::STBoundsDecider(const TaskConfig& config)
    : Decider(config) {
  CHECK(config.has_st_bounds_decider_config());
  st_bounds_config_ = config.st_bounds_decider_config();
}

Status STBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  const PathData &path_data = reference_line_info->path_data();
  PathDecision *const path_decision = reference_line_info->path_decision();

  STObstaclesProcessor st_obstacles_processor(
      path_data.discretized_path().Length(), st_bounds_config_.total_time(),
      path_data);
  st_obstacles_processor.MapObstaclesToSTBoundaries(path_decision);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
