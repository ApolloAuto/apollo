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

#pragma once

#include <vector>

#include "modules/common_msgs/prediction_msgs/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class ValidationChecker {
 public:
  /**
   * @brief Compute the probability by centripetal acceleration
   * @param lane sequence
   * @param current speed of obstacle
   * @return probability
   */
  static double ProbabilityByCentripetalAcceleration(
      const LaneSequence& lane_sequence, const double speed);

  /**
   * @brief Check the validity of trajectory's centripetal acceleration
   * @param The discretized trajectory
   * @return The validity of trajectory's centripetal acceleration
   */
  static bool ValidCentripetalAcceleration(
      const std::vector<common::TrajectoryPoint>& discretized_trajectory);

  /**
   * @brief Check if a trajectory point is valid
   * @param A trajectory point
   * @return True if the trajectory point is valid
   */
  static bool ValidTrajectoryPoint(
      const common::TrajectoryPoint& trajectory_point);

 private:
  ValidationChecker() = delete;
};

}  // namespace prediction
}  // namespace apollo
