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
 * @brief Define the predictor base class
 */

#pragma once

#include <vector>

#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class Predictor {
 public:
  /**
   * @brief Constructor
   */
  Predictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~Predictor() = default;

  /**
   * @brief Get prediction trajectories
   */
  virtual const std::vector<Trajectory>& trajectories();

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  virtual void Predict(Obstacle* obstacle) = 0;

  /**
   * @brief Get trajectory size
   * @return Size of trajectories
   */
  size_t NumOfTrajectories();

  /**
   * @brief Clear all trajectories
   */
  virtual void Clear();

  /**
   * @brief Trim prediction trajectories by adc trajectory
   * @param ADC trajectory container
   */
  void TrimTrajectories(const Obstacle* obstacle,
                        const ADCTrajectoryContainer* adc_trajectory_container);

 protected:
  /**
   * @brief Generate trajectory from trajectory points
   * @param A vector of trajectory points
   * @return Generated trajectory
   */
  static Trajectory GenerateTrajectory(
      const std::vector<apollo::common::TrajectoryPoint>& points);

  /**
   * @brief Set equal probability to prediction trajectories
   * @param probability total probability
   * @param start_index The start index to set equal probability
   */
  void SetEqualProbability(const double probability, const size_t start_index);

  /**
   * @brief Trim a single prediction trajectory,
   *        keep the portion that is not in junction.
   * @param adc_segments trajectory segments of ADC trajectory
   * @param trajectory The trimed prediction trajectory
   * @return If the prediction trajectory is trimed
   */
  bool TrimTrajectory(const Obstacle* obstacle,
                      const ADCTrajectoryContainer* adc_trajectory_container,
                      Trajectory* trajectory);

  /**
   * @brief Determine if an obstacle is supposed to stop within a distance
   * @param The latest feature of obstacle
   * @param The distance to stop
   * @param The output param of acceleration
   * @return If the obstacle is supposed to stop within a distance
   */
  bool SupposedToStop(const Feature& feature, const double stop_distance,
                      double* acceleration);

 protected:
  std::vector<Trajectory> trajectories_;
};

}  // namespace prediction
}  // namespace apollo
