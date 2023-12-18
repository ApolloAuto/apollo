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

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "modules/planning/tasks/path_reference_decider/proto/path_reference_decider.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/path_boundary.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {
class PathReferenceDecider : public Task {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;

  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(Frame *frame,
                                 ReferenceLineInfo *reference_line_info);

  /**
   * @brief check is learning model output is within path bounds
   *
   * @param path_reference learning model output
   * @param path_bound path boundaries for rule-based model
   * @return true using learning model output as path reference
   * @return false
   */
  bool IsValidPathReference(
      const ReferenceLineInfo &reference_line_info,
      const PathBoundary &path_bound,
      const std::vector<common::PathPoint> &path_reference);

  /**
   * @brief convert discrete path bounds to line segments
   *
   */
  void PathBoundToLineSegments(
      const PathBoundary &path_bound,
      std::vector<std::vector<common::math::LineSegment2d>>
          *path_bound_segments);

  /**
   * @brief convert path points from evenly dt to evenly ds distribution
   *
   * @param path_bound
   * @param path_reference
   * @param evaluated_path_reference
   */
  void EvaluatePathReference(
      const PathBoundary &path_bound,
      const std::vector<common::PathPoint> &path_reference,
      std::vector<common::PathPoint> *evaluated_path_reference);

  /**
   * @brief check if a point (x,y) is within path bounds
   *
   * @param reference_line_info
   * @param path_bound
   * @param x
   * @param y
   * @return int
   */

  int IsPointWithinPathBounds(const ReferenceLineInfo &reference_line_info,
                              const PathBoundary &path_bound, const double x,
                              const double y);

  /**
   * @brief Get the Regular Path Bound object
   *
   * @param path_bounds
   * @return size_t
   */
  size_t GetRegularPathBound(
      const std::vector<PathBoundary> &path_bounds) const;

  /**
   * @brief check is ADC box along path reference is within path bounds
   *        more accurate method.
   *
   * @param path_reference
   * @param regular_path_bound
   * @return true
   * @return false
   */
  bool IsADCBoxAlongPathReferenceWithinPathBounds(
      const std::vector<common::TrajectoryPoint> &path_reference,
      const PathBoundary &regular_path_bound);

  void ConvertTrajectoryToPath(
      const std::vector<common::TrajectoryPoint> &trajectory,
      std::vector<common::PathPoint> *path);

  void RecordDebugInfo(const std::vector<common::PathPoint> &path_points,
                       const std::string &path_name,
                       ReferenceLineInfo *const reference_line_info);

 private:
  static int valid_path_reference_counter_;  // count valid path reference
  static int total_path_counter_;            // count total path
  PathReferenceDeciderConfig config_;        // the config the task
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PathReferenceDecider,
                                     Task)

}  // namespace planning
}  // namespace apollo
