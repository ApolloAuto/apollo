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
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/macros.h"

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/pnc_map/path.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning_status.pb.h"
#include "modules/routing/proto/routing.pb.h"

/**
 * @brief PlanningContext is the runtime context in planning. It is
 * persistent across multiple frames.
 */
namespace apollo {
namespace planning {

class PlanningContext {
 public:
  struct ProceedWithCautionSpeedParam {
    bool is_fixed_distance = false;
    double distance = 5.0;  // m
  };

  // scenario context
  struct ScenarioInfo {
    /////////////////////////
    // general info, set up by ScenarioManager::Observe()
    // all traffic lights ahead, with signal info
    std::unordered_map<std::string, const apollo::perception::TrafficLight*>
        traffic_lights;

    /////////////////////////
    // scenario specific info, set up by ScenarioManager::ScenarioDispatch()
    // current stop sign
    apollo::hdmap::PathOverlap current_stop_sign_overlap;
    // current traffic light (vector)
    std::vector<apollo::hdmap::PathOverlap> current_traffic_light_overlaps;

    // still in the scenario for this overlap, but stop already done
    // => no stop fence from decider_rule_based_stop task
    std::vector<std::string> stop_done_overlap_ids;

    ProceedWithCautionSpeedParam proceed_with_caution_speed;
    std::vector<std::string> stop_sign_wait_for_obstacles;

    // TODO(all): to be removed when SidePass obstacle decision impl is done
    std::string side_pass_front_blocking_obstacle_id;
  };

  // @brief a container logging the data required for non-scenario side pass
  // functionality
  struct SidePassInfo {
    bool change_lane_stop_flag = false;
    common::PathPoint change_lane_stop_path_point;
  };

  static void Clear();

  static void Init();

  static const PlanningStatus& Planningstatus() { return planning_status_; }

  static PlanningStatus* MutablePlanningStatus() { return &planning_status_; }

  static ScenarioInfo* GetScenarioInfo() { return &scenario_info_; }

  static const SidePassInfo& side_pass_info() { return side_pass_info_; }

  static SidePassInfo* mutable_side_pass_info() { return &side_pass_info_; }

 private:
  static PlanningStatus planning_status_;
  static ScenarioInfo scenario_info_;
  static SidePassInfo side_pass_info_;

  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext)
};

}  // namespace planning
}  // namespace apollo
