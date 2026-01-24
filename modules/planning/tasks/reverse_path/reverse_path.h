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
#include <vector>
#include "modules/planning/tasks/reverse_path/proto/reverse_path.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

namespace apollo {
namespace planning {

class ReversePath : public PathGeneration {
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
  bool DecidePathBounds(PathBoundary* boundary, double& reference_line_backward_length);
  /**
   * @brief Optimize paths for each path boundary
   * @param path_boundaries is input path boundaries
   * @param candidate_path_data is output paths
   */
  bool OptimizePath(PathBoundary* path_boundary, PathData* candidate_path_data);
  bool OptimizePathOsqp(PathBoundary& path_boundary,
                               PathData* candidate_path_data);
  bool InitPathBoundary(PathBoundary* const path_bound, SLState init_sl_state, double& reference_line_backward_length);
  bool GetBoundaryFromSquare(const ReferenceLineInfo& reference_line_info,
                             PathBoundary* const path_boundary,
                             const SLState& init_sl_state);
 private:
  hdmap::PathOverlap junction_overlap_;
  ReversePathConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ReversePath, Task)
}  // namespace planning
}  // namespace apollo
