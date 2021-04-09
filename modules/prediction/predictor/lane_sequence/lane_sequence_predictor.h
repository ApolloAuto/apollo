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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Define lane sequence predictor
 */

#ifndef MODULES_PREDICTION_PREDICTOR_LANE_SEQUENCE_LANE_SEQUENCE_PREDICTOR_H_
#define MODULES_PREDICTION_PREDICTOR_LANE_SEQUENCE_LANE_SEQUENCE_PREDICTOR_H_

#include <string>
#include <vector>

#include "modules/common/math/kalman_filter.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/prediction/predictor/sequence/sequence_predictor.h"
#include "modules/prediction/proto/lane_graph.pb.h"

namespace apollo {
namespace prediction {

class LaneSequencePredictor : public SequencePredictor {
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
   * @brief Draw lane sequence trajectory points
   * @param Obstacle
   * @param Lane sequence
   * @param Total prediction time
   * @param Prediction period
   * @param A vector of generated trajectory points
   */
  void DrawLaneSequenceTrajectoryPoints(
      const Obstacle& obstacle, const LaneSequence& lane_sequence,
      const double total_time, const double period,
      std::vector<apollo::common::TrajectoryPoint>* points);
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_PREDICTOR_VEHICLE_LANE_SEQUENCE_PREDICTOR_H_
