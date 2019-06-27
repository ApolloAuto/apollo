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
 * @brief Define move sequence predictor
 */

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/prediction/predictor/sequence/sequence_predictor.h"

namespace apollo {
namespace prediction {

class MoveSequencePredictor : public SequencePredictor {
 public:
  /**
   * @brief Constructor
   */
  MoveSequencePredictor();

  /**
   * @brief Destructor
   */
  virtual ~MoveSequencePredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

  FRIEND_TEST(MoveSequencePredictorTest, Polynomial);
  FRIEND_TEST(MoveSequencePredictorTest, Utils);

 private:
  bool DrawMoveSequenceTrajectoryPoints(
      const Obstacle& obstacle, const LaneSequence& lane_sequence,
      const double total_time, const double period,
      std::vector<apollo::common::TrajectoryPoint>* points);

  std::pair<double, double> ComputeLonEndState(
      const std::array<double, 3>& init_s, const LaneSequence& lane_sequence);

  double ComputeTimeToLatEndConditionByVelocity(
      const Obstacle& obstacle, const LaneSequence& lane_sequence);

  std::vector<double> GenerateCandidateTimes();

  double CostFunction(const double max_lat_acc, const double time_to_end_state,
                      const double time_to_lane_edge,
                      const double bell_curve_mu);
};

}  // namespace prediction
}  // namespace apollo
