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

#ifndef MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLES_H_
#define MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLES_H_

#include <mutex>

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
  ObstaclesContainer() = default;

  /**
   * @brief Destructor
   */
  virtual ~ObstaclesContainer() = default;

  /**
   * @brief Insert a data message into the container
   * @param Data message to be inserted in protobuf
   */
  void Insert(const ::google::protobuf::Message& message) override;

  Obstacle* GetObstacle(int id);

 private:
  void InsertPerceptionObstacle(
      const apollo::perception::PerceptionObstacle& perception_obstacle,
      const double timestamp);

  bool IsPredictable(
      const apollo::perception::PerceptionObstacle& perception_obstacle);

  void SetObstacleLaneGraphFeatures(
      const apollo::perception::PerceptionObstacles& perception_obstacles);

 private:
  apollo::common::util::LRUCache<int, Obstacle> obstacles_;
  static std::mutex g_mutex_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_OBSTACLES_OBSTACLES_H_
