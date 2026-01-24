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
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/park_data_center/park_data_center.h"
#include "modules/planning/park_data_center/util/lane_escape_util.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_base/reference_line/reference_point.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/lane_follow_park/lane_follow_park_scenario.h"
#include "modules/planning/scenarios/lane_follow_park/proto/lane_follow_park_scenario.pb.h"

namespace apollo {
namespace planning {

class LaneFollowParkStage : public Stage {
 public:
 bool Init(
            const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir,
            void* context);

  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

  StageResult PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);

  void PlanFallbackTrajectory(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);

  common::SLPoint GetStopSL(const ObjectStop& stop_decision,
                            const ReferenceLine& reference_line) const;

  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);

 private:
  bool IsNeedEscape(const ReferenceLineInfo& reference_line_info);
  bool IsEnoughSpace(const ReferenceLineInfo& reference_line_info,
        const std::string& blocking_obstacle_id);
  bool IsQueueSence(const ReferenceLineInfo& reference_line_info, const std::string& blocking_obstacle_id);
  bool IsStableBlockObs(const std::string& blocking_obstacle_id);
  void GetClosestStopDecisionObs(const ReferenceLineInfo& reference_line_info, std::string& stop_decision_obs_id, double& min_distance);

  ScenarioLaneFollowParkConfig scenario_config_;
  std::pair<std::shared_ptr<Obstacle>, int> queue_sence_obs_info_;
  std::pair<std::string, int> stable_block_obs_count_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LaneFollowParkStage, Stage)

}  // namespace planning
}  // namespace apollo
