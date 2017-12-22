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

#ifndef MODULES_PREDICTION_PREDICTOR_PREDICTOR_H_
#define MODULES_PREDICTION_PREDICTOR_PREDICTOR_H_

#include <vector>

#include "modules/common/math/line_segment2d.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/prediction/container/obstacles/obstacle.h"

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
  int NumOfTrajectories();

  /**
   * @brief Clear all trajectories
   */
  virtual void Clear();

  /**
   * @brief Trim prediction trajectories by adc trajectory
   * @param ADC trajectory container
   */
  void TrimTrajectories(
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
  void SetEqualProbability(double probability, int start_index);

  /**
   * @brief Trim a single prediction trajectory
   * @param adc_segments trajectory segments of ADC trajectory
   * @param trajectory The trimed prediction trajectory
   * @return If the prediction trajectory is trimed
   */
  bool TrimTrajectory(
      const ADCTrajectoryContainer* adc_trajectory_container,
      Trajectory* trajectory);

 protected:
  std::vector<Trajectory> trajectories_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTOR_PREDICTOR_H_
