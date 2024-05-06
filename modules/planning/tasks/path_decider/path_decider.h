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
 * @file
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/tasks/path_decider/proto/path_decider.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

class PathDecider : public Task {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;

  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(const ReferenceLineInfo *reference_line_info,
                                 const PathData &path_data,
                                 PathDecision *const path_decision);

  bool MakeObjectDecision(const PathData &path_data,
                          const std::string &blocking_obstacle_id,
                          PathDecision *const path_decision);

  bool MakeStaticObstacleDecision(const PathData &path_data,
                                  const std::string &blocking_obstacle_id,
                                  PathDecision *const path_decision);
  bool IgnoreBackwardObstacle(PathDecision *const path_decision);
  ObjectStop GenerateObjectStopDecision(const Obstacle &obstacle) const;

 private:
  PathDeciderConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::PathDecider, Task)

}  // namespace planning
}  // namespace apollo
