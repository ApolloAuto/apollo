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
 **/

#ifndef MODULES_PREDICTION_COMMON_VALIDATION_CHECKER_H_
#define MODULES_PREDICTION_COMMON_VALIDATION_CHECKER_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class ValidationChecker {
 public:
  /**
   * @brief Compute the probability by centripedal acceleration
   * @param lane sequence
   * @param current speed of obstacle
   * @return probability
   */
  static double ProbabilityByCentripedalAcceleration(
      const LaneSequence& lane_sequence, const double speed);

  /**
   * @brief Check the validity of trajectory's centripedal acceleration
   * @param trajectory_points The input trajectory points
   * @return The validity of trajectory's centripedal acceleration
   */
  static bool ValidCentripedalAcceleration(
      const std::vector<::apollo::common::TrajectoryPoint>& trajectory_points);

  /**
   * @brief Check if a trajectory point is valid
   * @param A trajectory point
   * @return True if the trajectory point is valid
   */
  static bool ValidTrajectoryPoint(
      const ::apollo::common::TrajectoryPoint& trajectory_point);

 private:
  ValidationChecker() = delete;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_COMMON_VALIDATION_CHECKER_H_
