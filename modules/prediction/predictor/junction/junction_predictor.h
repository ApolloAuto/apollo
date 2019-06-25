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
 * @brief Define junction predictor
 */

#pragma once

#include <string>
#include <vector>

#include "modules/prediction/predictor/predictor.h"

namespace apollo {
namespace prediction {

class JunctionPredictor : public Predictor {
 public:
  /**
   * @brief Constructor
   */
  JunctionPredictor();

  /**
   * @brief Destructor
   */
  virtual ~JunctionPredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

 private:
  void DrawJunctionTrajectoryPoints(
      const Feature& feature, const JunctionExit& junction_exit,
      const double total_time, const double period,
      std::vector<apollo::common::TrajectoryPoint>* trajectory_points);

  std::vector<JunctionExit> MostLikelyJunctions(const Feature& feature);

  /**
   * @brief Get best pass time
   * @param start_x, end_x, start_y, end_y
   */
  double GetBestTime(const std::array<double, 2>& start_x,
                     const std::array<double, 2>& end_x,
                     const std::array<double, 2>& start_y,
                     const std::array<double, 2>& end_y);

  double CostFunction(const std::array<double, 4>& x_coeffs,
                      const std::array<double, 4>& y_coeffs,
                      const double time_to_exit);

  std::vector<double> GenerateCandidateTimes();
};

}  // namespace prediction
}  // namespace apollo
