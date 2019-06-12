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

#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/container/container.h"

namespace apollo {
namespace prediction {

class PoseContainer : public Container {
 public:
  /**
   * @brief Constructor
   */
  PoseContainer() = default;

  /**
   * @brief Destructor
   */
  virtual ~PoseContainer() = default;

  /**
   * @brief Insert a data message into the container
   * @param Data message to be inserted in protobuf
   */
  void Insert(const ::google::protobuf::Message& message) override;

  /**
   * @brief Transform pose to a perception obstacle.
   * @return A pointer to a perception obstacle.
   */
  const perception::PerceptionObstacle* ToPerceptionObstacle();

  /**
   * @brief Get timestamp
   */
  double GetTimestamp();

 private:
  /**
   * @brief Update pose
   * @param Received localization message
   */
  void Update(const localization::LocalizationEstimate& localization);

 public:
  static const perception::PerceptionObstacle::Type type_ =
      perception::PerceptionObstacle::VEHICLE;

 private:
  std::unique_ptr<perception::PerceptionObstacle> obstacle_ptr_;
};

}  // namespace prediction
}  // namespace apollo
