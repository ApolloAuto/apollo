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

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/macros.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/pnc_map/path.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/proto/path_decider_info.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

/**
 * @brief PlanningContext is the runtime context in planning. It is
 * persistent across multiple frames.
 */
namespace apollo {
namespace planning {

constexpr int kPathScenarioTransitionHysteresisFrame = 5;

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

  const SidePassInfo& side_pass_info() { return side_pass_info_; }

  SidePassInfo* mutable_side_pass_info() { return &side_pass_info_; }

  struct FallBackInfo {
    std::string last_successful_path_label;
  };

  const FallBackInfo& fallback_info() { return fallback_info_; }

  FallBackInfo* mutable_fallback_info() { return &fallback_info_; }

  struct OpenSpaceInfo {
    std::vector<std::string> partitioned_trajectories_index_history;
  };

  const OpenSpaceInfo& open_space_info() { return open_space_info_; }

  OpenSpaceInfo* mutable_open_space_info() { return &open_space_info_; }

  void Clear();

  void Init();

  const PlanningStatus& planning_status() { return planning_status_; }

  PlanningStatus* mutable_planning_status() { return &planning_status_; }

  const PathDeciderInfo& path_decider_info() { return path_decider_info_; }

  PathDeciderInfo* mutable_path_decider_info() { return &path_decider_info_; }

  /////////////////////////////////////////////////////////////////////////////
  void IncrementFrontStaticObstacleCycleCounter() {
    front_static_obstacle_cycle_counter_ =
        std::min(front_static_obstacle_cycle_counter_ + 1,
                 kPathScenarioTransitionHysteresisFrame);
  }

  void DecrementFrontStaticObstacleCycleCounter() {
    front_static_obstacle_cycle_counter_ =
        std::max(front_static_obstacle_cycle_counter_ - 1,
                 -kPathScenarioTransitionHysteresisFrame);
  }

  void ResetFrontStaticObstacleCycleCounter() {
    front_static_obstacle_cycle_counter_ = 0;
  }

  int front_static_obstacle_cycle_counter() {
    return front_static_obstacle_cycle_counter_;
  }

  void set_front_static_obstacle_id(
      const std::string& front_static_obstacle_id) {
    front_static_obstacle_id_ = front_static_obstacle_id;
  }

  std::string front_static_obstacle_id() { return front_static_obstacle_id_; }

  /////////////////////////////////////////////////////////////////////////////
  void IncrementAbleToUseSelfLaneCounter() {
    able_to_use_self_lane_counter_ =
        std::min(able_to_use_self_lane_counter_ + 1,
                 kPathScenarioTransitionHysteresisFrame);
  }

  void DecrementAbleToUseSelfLaneCounter() {
    able_to_use_self_lane_counter_ =
        std::max(able_to_use_self_lane_counter_ - 1,
                 -kPathScenarioTransitionHysteresisFrame);
  }

  void ResetAbleToUseSelfLaneCounter() { able_to_use_self_lane_counter_ = 0; }

  int able_to_use_self_lane_counter() { return able_to_use_self_lane_counter_; }

  /////////////////////////////////////////////////////////////////////////////
  void set_is_in_path_lane_borrow_scenario(
      bool is_in_path_lane_borrow_scenario) {
    is_in_path_lane_borrow_scenario_ = is_in_path_lane_borrow_scenario;
  }

  bool is_in_path_lane_borrow_scenario() {
    return is_in_path_lane_borrow_scenario_;
  }

 private:
  PlanningStatus planning_status_;
  SidePassInfo side_pass_info_;
  FallBackInfo fallback_info_;
  OpenSpaceInfo open_space_info_;
  PathDeciderInfo path_decider_info_;

  int front_static_obstacle_cycle_counter_ = 0;
  std::string front_static_obstacle_id_ = "";
  int able_to_use_self_lane_counter_ = 0;

  bool is_in_path_lane_borrow_scenario_ = false;

  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext)
};

}  // namespace planning
}  // namespace apollo
