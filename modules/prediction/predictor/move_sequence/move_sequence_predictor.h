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

#ifndef MODULES_PREDICTION_PREDICTOR_MOVE_SEQUENCE_MOVE_SEQUENCE_PREDICTOR_H_
#define MODULES_PREDICTION_PREDICTOR_MOVE_SEQUENCE_MOVE_SEQUENCE_PREDICTOR_H_

#include <array>
#include <string>
#include <vector>
#include <utility>
#include "Eigen/Dense"

#include "modules/common/math/kalman_filter.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/prediction/predictor/sequence/sequence_predictor.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class MoveSequencePredictor : public SequencePredictor {
 public:
  /**
   * @brief Constructor
   */
  MoveSequencePredictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~MoveSequencePredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

 private:
  void DrawMoveSequenceTrajectoryPoints(
      const Obstacle& obstacle, const LaneSequence& lane_sequence,
      const double total_time, const double period,
      std::vector<apollo::common::TrajectoryPoint>* points);

  void GetLongitudinalPolynomial(
      const Obstacle& obstacle, const LaneSequence& lane_sequence,
      std::pair<double, double>* lon_end_state,
      std::array<double, 5>* coefficients);

  void GetLateralPolynomial(const Obstacle& obstacle,
                            const LaneSequence& lane_sequence,
                            const double time_to_end_state,
                            std::array<double, 6>* coefficients);

  double ComputeTimeToLatEndConditionByVelocity(
      const Obstacle& obstacle, const LaneSequence& lane_sequence);

  std::pair<double, double> ComputeLonEndState(
      const std::array<double, 3>& init_s,
      const LaneSequence& lane_sequence);

  void GenerateCandidateTimes(std::vector<double>* candidate_times);
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTOR_MOVE_SEQUENCE_MOVE_SEQUENCE_PREDICTOR_H_
