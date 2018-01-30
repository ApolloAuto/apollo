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

/**
 * @file
 * @brief Define pedestrian predictor
 */

#ifndef MODULES_PREDICTION_PREDICTOR_REGIONAL_REGIONAL_PREDICTOR_H_
#define MODULES_PREDICTION_PREDICTOR_REGIONAL_REGIONAL_PREDICTOR_H_

#include <vector>

#include "Eigen/Dense"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/prediction/predictor/predictor.h"

namespace apollo {
namespace prediction {

class RegionalPredictor : public Predictor {
 public:
  /**
   * @brief Constructor
   */
  RegionalPredictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~RegionalPredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

  void GenerateStillTrajectory(const Obstacle* obstacle, double probability);

  void GenerateMovingTrajectory(const Obstacle* obstacle, double probability);

 private:
  void DrawStillTrajectory(
      const Eigen::Vector2d& position, const double heading, const double speed,
      const double total_time,
      std::vector<apollo::common::TrajectoryPoint>* points);

  void DrawMovingTrajectory(
      const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
      const Eigen::Vector2d& acceleration,
      const apollo::common::math::KalmanFilter<double, 2, 2, 4>& kf,
      const double total_time,
      std::vector<apollo::common::TrajectoryPoint>* left_points,
      std::vector<apollo::common::TrajectoryPoint>* right_points);

  void GetTrajectoryCandidatePoints(
      const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
      const Eigen::Vector2d& acceleration,
      const apollo::common::math::KalmanFilter<double, 2, 2, 4>& kf,
      const double total_time,
      std::vector<apollo::common::TrajectoryPoint>* middle_points,
      std::vector<apollo::common::TrajectoryPoint>* boundary_points);

  void UpdateTrajectoryPoints(
      const apollo::common::TrajectoryPoint& starting_point,
      const Eigen::Vector2d& velocity, const double delta_ts,
      const std::vector<apollo::common::TrajectoryPoint>& middle_points,
      const std::vector<apollo::common::TrajectoryPoint>& boundary_points,
      std::vector<apollo::common::TrajectoryPoint>* left_points,
      std::vector<apollo::common::TrajectoryPoint>* right_points);

  void InsertTrajectoryPoint(
      const apollo::common::TrajectoryPoint& prev_middle_point,
      const Eigen::Vector2d& middle_direction,
      const apollo::common::TrajectoryPoint& boundary_point, const double speed,
      const double delta_ts, int* left_i, int* right_i, double* left_heading,
      double* right_heading,
      std::vector<apollo::common::TrajectoryPoint>* left_points,
      std::vector<apollo::common::TrajectoryPoint>* right_points);

  void GetTwoEllipsePoints(const double position_x, const double position_y,
                           const double direction_x, const double direction_y,
                           const double ellipse_len_x,
                           const double ellipse_len_y,
                           apollo::common::TrajectoryPoint* ellipse_point_1,
                           apollo::common::TrajectoryPoint* ellipse_point_2);

  void GetQuadraticCoefficients(const double position_x,
                                const double position_y,
                                const double direction_x,
                                const double direction_y,
                                const double ellipse_len_1,
                                const double ellipse_len_2,
                                std::vector<double>* coefficients);

  void UpdateHeading(const apollo::common::TrajectoryPoint& curr_point,
                     std::vector<apollo::common::TrajectoryPoint>* points);
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTOR_PEDESTRIAN_REGIONAL_PREDICTOR_H_
