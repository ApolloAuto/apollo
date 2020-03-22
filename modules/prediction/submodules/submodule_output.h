/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @brief Output information of prediction container submodule
 */

#pragma once

#include <vector>

#include "absl/time/time.h"

#include "modules/common/util/lru_cache.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/container/obstacles/obstacle.h"

namespace apollo {
namespace prediction {

class SubmoduleOutput {
 public:
  /**
   * @brief Constructor
   */
  SubmoduleOutput() = default;

  /**
   * @brief Destructor
   */
  virtual ~SubmoduleOutput() = default;

  void InsertObstacle(const Obstacle&& obstacle);

  void InsertEgoVehicle(const Obstacle&& ego_vehicle);

  void set_curr_frame_movable_obstacle_ids(
      const std::vector<int>& curr_frame_movable_obstacle_ids);

  void set_curr_frame_unmovable_obstacle_ids(
      const std::vector<int>& curr_frame_unmovable_obstacle_ids);

  void set_curr_frame_considered_obstacle_ids(
      const std::vector<int>& curr_frame_considered_obstacle_ids);

  void set_frame_start_time(const absl::Time& frame_start_time);

  const std::vector<Obstacle>& curr_frame_obstacles() const;

  const Obstacle& GetEgoVehicle() const;

  const std::vector<apollo::perception::PerceptionObstacle>&
  curr_frame_perception_obstacles() const;

  std::vector<int> curr_frame_movable_obstacle_ids() const;

  std::vector<int> curr_frame_unmovable_obstacle_ids() const;

  std::vector<int> curr_frame_considered_obstacle_ids() const;

  const absl::Time& frame_start_time() const;

 protected:
  std::vector<Obstacle> curr_frame_obstacles_;
  Obstacle ego_vehicle_;
  std::vector<int> curr_frame_movable_obstacle_ids_;
  std::vector<int> curr_frame_unmovable_obstacle_ids_;
  std::vector<int> curr_frame_considered_obstacle_ids_;
  absl::Time frame_start_time_;
};

}  // namespace prediction
}  // namespace apollo
