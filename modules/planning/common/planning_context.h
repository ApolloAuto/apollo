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

#include "cyber/common/macros.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

/**
 * @brief PlanningContext is the runtime context in planning. It is
 * persistent across multiple frames.
 */
namespace apollo {
namespace planning {

class PlanningContext {
 public:
  // TODO(jinyun): to be removed/cleaned up.
  //               put all of them inside Planningstatus
  // @brief a container logging the data required for non-scenario side pass
  // functionality
  struct SidePassInfo {
    bool change_lane_stop_flag = false;
    common::PathPoint change_lane_stop_path_point;
    bool check_clear_flag = false;
  };
  static const SidePassInfo& side_pass_info() { return side_pass_info_; }
  static SidePassInfo* mutable_side_pass_info() { return &side_pass_info_; }

  struct LaneFollowInfo {
    bool blocked_by_front_obstacle = false;
    std::string front_obstacle_id = "";
    double front_obstacle_distance = 0.0;
    int num_of_blocked_cycles = 0;
  };

  static void Clear();

  static void Init();

  static const PlanningStatus& Planningstatus() { return planning_status_; }

  static PlanningStatus* MutablePlanningStatus() { return &planning_status_; }

  static ScenarioInfo* GetScenarioInfo() { return &scenario_info_; }

  static const SidePassInfo& side_pass_info() { return side_pass_info_; }

  static SidePassInfo* mutable_side_pass_info() { return &side_pass_info_; }

  static const LaneFollowInfo& lane_follow_info() { return lane_follow_info_; }

  static LaneFollowInfo* mutable_lane_follow_info() { return &lane_follow_info_; }

 private:
  static PlanningStatus planning_status_;
  static SidePassInfo side_pass_info_;
  static LaneFollowInfo lane_follow_info_;

  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext)
};

}  // namespace planning
}  // namespace apollo
