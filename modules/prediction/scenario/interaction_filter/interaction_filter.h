/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "cyber/common/macros.h"

#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/scenario/scenario_features/cruise_scenario_features.h"
#include "modules/prediction/scenario/scenario_features/scenario_features.h"

namespace apollo {
namespace prediction {

class InteractionFilter {
 public:
  InteractionFilter() = delete;

  explicit InteractionFilter(
      const std::shared_ptr<ContainerManager>& container_manager);

  /**
   * @brief Assign interactive tag for vehicle type obstacles which are close to ADC
   */
  void AssignInteractiveTag();

 private:
   /**
   * @brief Assign interactive tag for obstacles in junction
   */
  void AssignInteractiveTagInJunction(const Obstacle& ego_vehicle,
                                    ObstaclesContainer* obstacles_container,
                                    const std::string& junction_id);

  /**
   * @brief Assign interactive tag for obstacles in the cruising lane 
   * which are in front of ADC
   */
  void AssignInteractiveTagCruiseKeepLane(
      const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container);

  /**
   * @brief Assign interactive tag for obstacles in the target lane
   * which are close to ADC(include backward obstacles) during lane change
   */
  void AssignInteractiveTagCruiseChangeLane(
      const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container);

  /**
   * @brief Assign interactive tag for obstacles that may interact with ADC 
   * according to the ADC's trajectory
   */
  void AssignInteractiveTagByEgoReferenceLine(
      const Obstacle& ego_vehicle, ObstaclesContainer* obstacles_container);

  void RankingInteractiveTagObstacles(const Obstacle& ego_vehicle,
                                    ObstaclesContainer* obstacles_container);

  void AssignInteractiveByMerge(
      const Obstacle& ego_vehicle,
      std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
      std::unordered_set<std::string>* const visited_lanes,
      ObstaclesContainer* obstacles_container);

  void AssignInteractiveByOverlap(
      const Obstacle& ego_vehicle,
      std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
      std::unordered_set<std::string>* const visited_lanes,
      ObstaclesContainer* obstacles_container);

  void SetInteractiveBackward(
      const double distance, const Obstacle& ego_vehicle,
      std::shared_ptr<const hdmap::LaneInfo> start_lane_info_ptr,
      std::unordered_set<std::string>* const visited_lanes,
      ObstaclesContainer* obstacles_container);

  void SetInteractiveIfCloseToEgo(const Obstacle& ego_vehicle,
                              const double distance_threshold,
                              Obstacle* obstacle_ptr);

 private:
  std::unordered_set<std::string> ego_back_lane_id_set_;

  std::shared_ptr<ContainerManager> container_manager_;

  std::string ego_lane_id_ = "";

  double ego_lane_s_ = 0.0;
};

}  // namespace prediction
}  // namespace apollo
