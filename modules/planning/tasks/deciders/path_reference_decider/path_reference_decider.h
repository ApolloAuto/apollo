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
 * @file path_reference_decider.h
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/planning/common/path_boundary.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {
class PathReferenceDecider : public Task {
 public:
  explicit PathReferenceDecider(const TaskConfig &config);

  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(Frame *frame,
                                 const ReferenceLineInfo *reference_line_info);

  /**
   * @brief check is learning model output is within path bounds
   *
   * @param path_reference learning model output
   * @param path_bound path boundaries for rule-based model
   * @return true using learning model output as path reference
   * @return false
   */
  bool IsValidPathReference(
      const std::vector<common::TrajectoryPoint> &path_reference,
      const std::vector<PathBoundary> &path_bound);
  /**
   * @brief convert discrete path bounds to line segments
   *
   */
  void PathBoundToLineSegments(
      const PathBoundary *path_bound,
      std::vector<std::vector<common::math::LineSegment2d>>
          &path_bound_segments);
};

}  // namespace planning
}  // namespace apollo
