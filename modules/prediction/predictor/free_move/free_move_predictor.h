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
 * @brief Define lane sequence predictor
 */

#pragma once

#include <vector>

#include "modules/prediction/predictor/predictor.h"

namespace apollo {
namespace prediction {

class FreeMovePredictor : public Predictor {
 public:
  /**
   * @brief Constructor
   */
  FreeMovePredictor();

  /**
   * @brief Destructor
   */
  virtual ~FreeMovePredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

 private:
  /**
   * @brief Generate free move trajectory
   * @param Position
   * @param Velocity
   * @param Acceleration
   * @param Kalman Filter
   * @param start time
   * @param Total time
   * @param Generated trajectory points
   */
  void DrawFreeMoveTrajectoryPoints(
      const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
      const Eigen::Vector2d& acc, const double theta, const double start_time,
      const double total_time, const double period,
      std::vector<apollo::common::TrajectoryPoint>* points);
};

}  // namespace prediction
}  // namespace apollo
