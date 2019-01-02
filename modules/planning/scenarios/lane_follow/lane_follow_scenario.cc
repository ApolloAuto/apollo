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

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"
#include "modules/planning/tasks/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/tasks/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/tasks/optimizers/path_decider/path_decider.h"
#include "modules/planning/tasks/optimizers/qp_piecewise_jerk_path/qp_piecewise_jerk_path_optimizer.h"
#include "modules/planning/tasks/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/tasks/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/tasks/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

std::unique_ptr<Stage> LaneFollowScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (stage_config.stage_type() != ScenarioConfig::LANE_FOLLOW_DEFAULT_STAGE) {
    AERROR << "Follow lane does not support stage type: "
           << ScenarioConfig::StageType_Name(stage_config.stage_type());
    return nullptr;
  }
  return std::unique_ptr<Stage>(new LaneFollowStage(stage_config));
}

bool LaneFollowScenario::IsTransferable(
    const Scenario& current_scenario, const common::TrajectoryPoint& ego_point,
    const Frame& frame) {
  // implement here
  if (current_scenario.scenario_type() == ScenarioConfig::LANE_FOLLOW) {
    return true;
  } else {
    return false;
  }
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
