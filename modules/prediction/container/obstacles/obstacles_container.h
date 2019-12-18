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
 * @brief Obstacles container
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common/util/lru_cache.h"
#include "modules/prediction/container/container.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/prediction/submodules/submodule_output.h"

namespace apollo {
namespace prediction {

class ObstaclesContainer : public Container {
 public:
  /**
   * @brief Constructor
   */
  ObstaclesContainer();

  /**
   * @brief Constructor from container output
   */
  explicit ObstaclesContainer(const SubmoduleOutput& submodule_output);

  /**
   * @brief Destructor
   */
  virtual ~ObstaclesContainer() = default;

  /**
   * @brief Insert a data message into the container
   * @param Data message to be inserted in protobuf
   */
  void Insert(const ::google::protobuf::Message& message) override;

  /**
   * @brief Insert an perception obstacle
   * @param Perception obstacle
   *        Timestamp
   */
  void InsertPerceptionObstacle(
      const perception::PerceptionObstacle& perception_obstacle,
      const double timestamp);

  /**
   * @brief Insert a feature proto message into the container
   * @param feature proto message
   */
  void InsertFeatureProto(const Feature& feature);

  /**
   * @brief Build lane graph for obstacles
   */
  void BuildLaneGraph();

  /**
   * @brief Build junction feature for obstacles
   */
  void BuildJunctionFeature();

  /**
   * @brief Get obstacle pointer
   * @param Obstacle ID
   * @return Obstacle pointer
   */
  Obstacle* GetObstacle(const int id);

  /**
   * @brief Clear obstacle container
   */
  void Clear();

  void CleanUp();

  size_t NumOfObstacles() { return ptr_obstacles_.size(); }

  const apollo::perception::PerceptionObstacle& GetPerceptionObstacle(
      const int id);

  /**
   * @brief Get movable obstacle IDs in the current frame
   * @return Movable obstacle IDs in the current frame
   */
  const std::vector<int>& curr_frame_movable_obstacle_ids();

  /**
   * @brief Get unmovable obstacle IDs in the current frame
   * @return unmovable obstacle IDs in the current frame
   */
  const std::vector<int>& curr_frame_unmovable_obstacle_ids();

  /**
   * @brief Get non-ignore obstacle IDs in the current frame
   * @return Non-ignore obstacle IDs in the current frame
   */
  const std::vector<int>& curr_frame_considered_obstacle_ids();

  /*
   * @brief Set non-ignore obstacle IDs in the current frame
   */
  void SetConsideredObstacleIds();

  /**
   * @brief Get current frame obstacle IDs in the current frame
   * @return Current frame obstacle IDs in the current frame
   */
  std::vector<int> curr_frame_obstacle_ids();

  double timestamp() const;

  SubmoduleOutput GetSubmoduleOutput(const size_t history_size,
                                     const absl::Time& frame_start_time);

 private:
  Obstacle* GetObstacleWithLRUUpdate(const int obstacle_id);

  /**
   * @brief Check if an obstacle is movable
   * @param An obstacle
   * @return True if the obstacle is movable; otherwise false;
   */
  bool IsMovable(const perception::PerceptionObstacle& perception_obstacle);

 private:
  double timestamp_ = -1.0;
  common::util::LRUCache<int, std::unique_ptr<Obstacle>> ptr_obstacles_;
  std::vector<int> curr_frame_movable_obstacle_ids_;
  std::vector<int> curr_frame_unmovable_obstacle_ids_;
  std::vector<int> curr_frame_considered_obstacle_ids_;
};

}  // namespace prediction
}  // namespace apollo
