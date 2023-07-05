/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/stage.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {

class LaneFollowStage : public Stage {
 public:
  LaneFollowStage(const ScenarioConfig::StageConfig& config,
                  const std::shared_ptr<DependencyInjector>& injector);

  StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);

  void PlanFallbackTrajectory(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,
                                   PathData* path_data);

  bool RetrieveLastFramePathProfile(
      const ReferenceLineInfo* reference_line_info, const Frame* frame,
      PathData* path_data);

  common::SLPoint GetStopSL(const ObjectStop& stop_decision,
                            const ReferenceLine& reference_line) const;

  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);

 private:
  ScenarioConfig config_;
  std::unique_ptr<Stage> stage_;
};

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
