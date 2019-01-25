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
#include <unordered_map>
#include <vector>

#include "modules/common/util/lru_cache.h"
#include "modules/prediction/container/container.h"
#include "modules/prediction/container/obstacles/obstacle.h"

namespace apollo {
namespace prediction {

class ObstaclesContainer : public Container {
 public:
  /**
   * @brief Constructor
   */
  ObstaclesContainer();

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

  /*
   * @brief Build current frame id mapping for all obstacles
   */
  void BuildCurrentFrameIdMapping(
      const perception::PerceptionObstacles& perception_obstacles);

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

  size_t NumOfObstacles() { return ptr_obstacles_.size(); }

  const apollo::perception::PerceptionObstacle&
  GetPerceptionObstacle(const int id);

  /**
   * @brief Get predictable obstacle IDs in the current frame
   * @return Predictable obstacle IDs in the current frame
   */
  const std::vector<int>& curr_frame_predictable_obstacle_ids();

  /**
   * @brief Get non-predictable obstacle IDs in the current frame
   * @return Non-predictable obstacle IDs in the current frame
   */
  const std::vector<int>& curr_frame_non_predictable_obstacle_ids();

  /**
   * @brief Get current frame obstacle IDs in the current frame
   * @return Current frame obstacle IDs in the current frame
   */
  std::vector<int> curr_frame_obstacle_ids();

 private:
  Obstacle* GetObstacleWithLRUUpdate(const int obstacle_id);
  /**
   * @brief Check if a perception_obstacle is an old existed obstacle
   * @param A PerceptionObstacle
   * @param An obstacle_ptr
   * @return True if the perception_obstacle is this obstacle; otherwise false;
   */
  bool AdaptTracking(const perception::PerceptionObstacle& perception_obstacle,
                      Obstacle* obstacle_ptr);

  /**
   * @brief Check if an obstacle is predictable
   * @param An obstacle
   * @return True if the obstacle is predictable; otherwise false;
   */
  bool IsPredictable(const perception::PerceptionObstacle& perception_obstacle);

  int PerceptionIdToPredictionId(const int perception_id);

 private:
  double timestamp_ = -1.0;
  common::util::LRUCache<int, std::unique_ptr<Obstacle>> ptr_obstacles_;
  // an id_mapping from perception_id to prediction_id
  common::util::LRUCache<int, int> id_mapping_;
  std::vector<int> curr_frame_predictable_obstacle_ids_;
  std::vector<int> curr_frame_non_predictable_obstacle_ids_;
  // perception_id -> prediction_id
  std::unordered_map<int, int> curr_frame_id_mapping_;
  // prediction_id -> perception_obstacle
  std::unordered_map<int, apollo::perception::PerceptionObstacle>
      curr_frame_id_perception_obstacle_map_;
};

}  // namespace prediction
}  // namespace apollo
