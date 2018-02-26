/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PREDICTION_CONTAINER_RELATIVE_OBSTACLES_RELATIVE_OBSTACLE_H_
#define MODULES_PREDICTION_CONTAINER_RELATIVE_OBSTACLES_RELATIVE_OBSTACLE_H_

#include <string>

#include "modules/perception/proto/perception_obstacle.pb.h"

namespace apollo {
namespace prediction {

class RelativeObstacle {
 public:
  /**
   * @brief Constructor
   */
  RelativeObstacle();

  /**
   * @brief Destructor
   */
  virtual ~RelativeObstacle();

  void Insert(const perception::PerceptionObstacle& perception_obstacle);

  int id() const;

  bool IsOnlane() const;

  std::string CurrentLaneId();
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_CONTAINER_RELATIVE_OBSTACLES_RELATIVE_OBSTACLE_H_
