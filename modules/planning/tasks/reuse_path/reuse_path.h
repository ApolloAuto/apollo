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
#include <utility>
#include <vector>
#include "modules/planning/tasks/reuse_path/proto/reuse_path.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

namespace apollo {
namespace planning {

class ReusePath : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool IsPathReusable(Frame* frame,
                      ReferenceLineInfo* const reference_line_info);

  void GetCurrentStopPositions(
      Frame* frame,
      std::vector<const common::PointENU*>* current_stop_positions);

  // get current s_projection of history objects which has stop decisions
  void GetHistoryStopPositions(
      ReferenceLineInfo* const reference_line_info,
      const std::vector<const HistoryObjectDecision*>&
          history_objects_decisions,
      std::vector<std::pair<const double, const common::PointENU>>*
          history_stop_positions);

  void GetADCSLPoint(const ReferenceLine& reference_line,
                     common::SLPoint* adc_position_sl);

  bool GetBlockingObstacleS(ReferenceLineInfo* const reference_line_info,
                            double* blocking_obstacle_s);
  // ignore blocking obstacle when it is far away
  bool IsIgnoredBlockingObstacle(ReferenceLineInfo* const reference_line_info);
  // check if path is collision free
  bool IsCollisionFree(ReferenceLineInfo* const reference_line_info);

  // check path length
  bool NotShortPath(const DiscretizedPath& current_path);

  // trim history path
  bool TrimHistoryPath(Frame* frame,
                       ReferenceLineInfo* const reference_line_info);

 private:
  ReusePathConfig config_;
  bool path_reusable_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ReusePath, Task)
}  // namespace planning
}  // namespace apollo
