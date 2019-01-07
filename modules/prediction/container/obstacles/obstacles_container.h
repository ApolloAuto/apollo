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

#include <string>
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
   * @brief Get predictable obstacle IDs in the current frame
   * @return Predictable obstacle IDs in the current frame
   */
  const std::vector<int>& GetCurrentFramePredictableObstacleIds() const;

  /**
   * @brief Clear obstacle container
   */
  void Clear();

 private:
  /**
   * @brief Check if an obstacle is predictable
   * @param An obstacle
   * @return True if the obstacle is predictable; otherwise false;
   */
  bool IsPredictable(const perception::PerceptionObstacle& perception_obstacle);

 private:
  double timestamp_ = -1.0;
  common::util::LRUCache<int, Obstacle> obstacles_;
  std::vector<int> curr_frame_predictable_obstacle_ids_;
};

}  // namespace prediction
}  // namespace apollo
