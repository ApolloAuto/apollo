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

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "modules/planning/tasks/pull_over_path/proto/pull_over_path.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

namespace apollo {
namespace planning {

using SLState = std::pair<std::array<double, 3>, std::array<double, 3>>;

class PullOverPath : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief Calculate all path boundaries
   * @param boundary is calculated path boundaries
   */
  bool DecidePathBounds(std::vector<PathBoundary>* boundary);
  /**
   * @brief Optimize paths for each path boundary
   * @param path_boundaries is input path boundaries
   * @param candidate_path_data is output paths
   */
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);
  /**
   * @brief Assess the feasibility of each path and select the best one
   * @param candidate_path_data is input paths
   * @param final_path is output the best path
   */
  bool AssessPath(std::vector<PathData>* candidate_path_data,
                  PathData* final_path);
  /**
   * @brief Using the left and right boundaries of road as output path_bound
   * @param reference_line_info is input data
   * @param path_bound is output the path boundary
   */
  bool GetBoundaryFromRoads(const ReferenceLineInfo& reference_line_info,
                            PathBoundary* const path_bound);
  /**
   * @brief Update left or right path bound with current lane boundary.
   * @param is_pull_over_right if true, adc will pull over at right side,and
   * update left path bound with left road boundary. Otherwise, adc will pull
   * over at left side, and update right path bound with right road boundary
   * @param path_bound is output path boundary
   */
  void UpdatePullOverBoundaryByLaneBoundary(bool is_pull_over_right,
                                            PathBoundary* const path_bound);
  /**
   * @brief Search a feasible pull over position
   * @param path_bound is input path boundary
   * @param pull_over_configuration is output  pull over position
   *    (pull over x, pull over y,
   *    pull over theta, pull over position  s-index in path bound)
   */
  bool SearchPullOverPosition(
      const PathBound& path_bound,
      std::tuple<double, double, double, int>* const pull_over_configuration);
  /**
   * @brief calculate nearest pull over s by vehicle kinematics model
   * @param pull_over_s is output pull over  position s
   */
  bool FindNearestPullOverS(double* pull_over_s);
  /**
   * @brief calculate pull over s by routing end
   * @param path_bound is input path boundary
   * @param pull_over_s is output pull over  position s
   */
  bool FindDestinationPullOverS(const PathBound& path_bound,
                                double* pull_over_s);

  PullOverPathConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PullOverPath, Task)
}  // namespace planning
}  // namespace apollo
