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

#ifndef MODULES_PREDICTION_PREDICTOR_VEHICLE_LANE_SEQUENCE_PREDICTOR_H_
#define MODULES_PREDICTION_PREDICTOR_VEHICLE_LANE_SEQUENCE_PREDICTOR_H_

#include <string>
#include <vector>
#include "Eigen/Dense"

#include "modules/prediction/predictor/predictor.h"
#include "modules/prediction/proto/lane_graph.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace prediction {

class LaneSequencePredictor : public Predictor {
 public:
  /**
   * @brief Constructor
   */
  LaneSequencePredictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~LaneSequencePredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

 protected:
  /**
   * @brief Clear private members
   */
  void Clear();

  /**
   * @brief Filter lane sequences
   * @param Lane graph
   * @param Current lane id
   * @param Vector of boolean indicating if a lane sequence is disqualified
   */
  void FilterLaneSequences(const LaneGraph& lane_graph,
                           const std::string& lane_id,
                           std::vector<bool> *enable_lane_sequence);

  /**
   * @brief Get lane change type
   * @param Current lane id
   * @param Lane sequence
   * @return Integer indicating lane change type:
   *         0: no lane change
   *         1: left lane change
   *         2: right lane change
   *        -1: other
   */
  int GetLaneChangeType(const std::string& lane_id,
                        const LaneSequence& lane_sequence);

  /**
   * @brief Get lane change distance with ADC
   * @param Target lane sequence
   * @return Lane change distance with ADC
   */
  double GetLaneChangeDistanceWithADC(const LaneSequence& lane_sequence);

  /**
   * @brief Get ADC status
   */
  void GetADC();

  /**
   * @brief Draw lane sequence trajectory points
   * @param Kalman filter
   * @param Lane sequence
   * @param Total prediction time
   * @param Prediction frequency
   * @param A vector of generated trajectory points
   */
  void DrawLaneSequenceTrajectoryPoints(
      const ::apollo::common::math::KalmanFilter<double, 4, 2, 0>& kf,
      const LaneSequence& sequence,
      double total_time,
      double freq,
      std::vector<::apollo::common::TrajectoryPoint> *points);

  /**
   * @brief Convert a lane sequence to string
   * @param Lane sequence
   * @return String describing the lane sequence
   */
  std::string ToString(const LaneSequence& sequence);

 private:
  std::string adc_lane_id_ = "";
  double adc_lane_s_ = 0.0;
  Eigen::Vector2d adc_position_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTOR_VEHICLE_LANE_SEQUENCE_PREDICTOR_H_
