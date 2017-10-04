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
#include <vector>
#include <string>
#include "Eigen/Dense"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/prediction/proto/lane_graph.pb.h"
#include "modules/prediction/predictor/predictor.h"
#include "modules/common/math/kalman_filter.h"

namespace apollo {
namespace prediction {

class MoveSequencePredictor : public Predictor {
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
  static const size_t COEFF_SIZE = 6;

  void DrawLaneSequenceTrajectoryPoints(
      const Obstacle& obstacle,
      const LaneSequence& lane_sequence,
      const double total_time, const double freq,
      std::vector<apollo::common::TrajectoryPoint>* points);

  void DrawManeuverTrajectoryPoints(
      const Obstacle& obstacle,
      const LaneSequence& lane_sequence,
      const double total_time, const double freq,
      std::vector<apollo::common::TrajectoryPoint>* points);

  void DrawMotionTrajectoryPoints(
      const Obstacle& obstacle,
      const double total_time, const double freq,
      std::vector<apollo::common::TrajectoryPoint>* points);

  void GetLongitudinalPolynomial(
      const Obstacle& obstacle,
      const LaneSequence& lane_sequence,
      const double time_to_lane_center,
      std::array<double, 5>* coefficients);

  void GetLateralPolynomial(
      const Obstacle& obstacle,
      const LaneSequence& lane_sequence,
      const double time_to_lane_center,
      std::array<double, 6>* coefficients);

  double Cost(const double t,
              const std::array<double, COEFF_SIZE>& coeffs,
              const double alpha);

  double MotionWeight(const double t);

  void FilterLaneSequences(const LaneGraph& lane_graph,
                           const std::string& lane_id,
                           std::vector<bool>* enable_lane_sequence);

  int GetLaneChangeType(const std::string& lane_id,
                        const LaneSequence& lane_sequence);

  double GetLaneChangeDistanceWithADC(const LaneSequence& lane_sequence);

  bool SameLaneSequence(const std::string& lane_id, double lane_s);

  void GetADC();

  std::string ToString(const LaneSequence& sequence);

  std::string adc_lane_id_ = "";
  double adc_lane_s_ = 0.0;
  Eigen::Vector2d adc_position_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTOR_MOVE_SEQUENCE_MOVE_SEQUENCE_PREDICTOR_H_
