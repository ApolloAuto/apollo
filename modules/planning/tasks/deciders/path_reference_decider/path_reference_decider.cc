/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @file path_reference_decider.cc
 */
#include "modules/planning/tasks/deciders/path_reference_decider/path_reference_decider.h"

#include <string>
#include <vector>

#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

PathReferenceDecider::PathReferenceDecider(const TaskConfig &config)
    : Task(config) {}

Status PathReferenceDecider::Execute(Frame *frame,
                                     ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Status::OK();
}

Status PathReferenceDecider::Process(
    const Frame *frame, const ReferenceLineInfo *reference_line_info) {
  // get path bounds info from reference line info
  const std::vector<PathBoundary> &path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";

  // get learning model output (trajectory) from frame
  const std::vector<common::TrajectoryPoint> &path_reference =
      frame->learning_data_adc_future_trajectory_points();
  ADEBUG << "There are " << path_reference.size() << " path points.";

  // check if trajectory points are within path bounds
  // if yes: use learning model output
  // otherwise: use previous path planning method
  // IsValidPathReference();

  // mark learning trajectory as path reference
  // MarkValidPathReference():

  // use learning model output
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
