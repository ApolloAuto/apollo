/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
 * @file lane_escape_util.h
 */

#pragma once

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class LaneEscapeUtil {
 public:
  static bool IsNeedEscape(ReferenceLineInfo& reference_line_info,
                  double min_distance_block_obs_to_junction,
                  bool enable_junction_borrow,
                  double passby_min_gap,
                  double passby_kappa_ratio,
                  int queue_check_count,
                  int stable_block_count);

  static bool IsEnoughSpace(
        const ReferenceLineInfo& reference_line_info,
        const std::string& blocking_obstacle_id,
        const double &passby_min_gap,
        const double &passby_kappa_ratio);

  static bool IsQueueSence(
          ReferenceLineInfo& reference_line_info,
          const std::string& blocking_obstacle_id,
          const int &queue_check_count,
          std::pair<std::shared_ptr<Obstacle>, int> &queue_obs_info);

  static bool IsStableBlockObs(ReferenceLineInfo& reference_line_info, const std::string& blocking_obstacle_id, const int &stable_block_count, std::pair<std::string, int> &stable_block_obs);

  static bool GetClosestStopDecisionObs(
          const ReferenceLineInfo& reference_line_info,
          std::string& stop_decision_obs_id,
          double& min_distance);
};

}  // namespace planning
}  // namespace apollo
