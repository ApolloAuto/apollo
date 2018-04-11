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
 * @brief Define the sequence predictor base class
 */

#ifndef MODULES_PREDICTION_PREDICTOR_SEQUENCE_SEQUENCE_PREDICTOR_H_
#define MODULES_PREDICTION_PREDICTOR_SEQUENCE_SEQUENCE_PREDICTOR_H_

#include <string>
#include <unordered_set>
#include <vector>

#include "Eigen/Dense"

#include "modules/prediction/proto/lane_graph.pb.h"

#include "modules/prediction/predictor/predictor.h"

namespace apollo {
namespace prediction {

class SequencePredictor : public Predictor {
 public:
  enum class LaneChangeType {
    LEFT,
    RIGHT,
    STRAIGHT,
    ONTO_LANE,
    INVALID,
  };

 public:
  /**
   * @brief Constructor
   */
  SequencePredictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~SequencePredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

 protected:
  /**
   * @brief Filter lane sequences
   * @param Lane graph
   * @param Current lane id
   * @param Vector of boolean indicating if a lane sequence is disqualified
   */
  void FilterLaneSequences(const Feature& feature, const std::string& lane_id,
                           std::vector<bool>* enable_lane_sequence);

  /**
   * @brief Get lane change type
   * @param Current lane id
   * @param Lane sequence
   * @return Integer indicating lane change type:
   */
  LaneChangeType GetLaneChangeType(const std::string& lane_id,
                                   const LaneSequence& lane_sequence);

  /**
   * @brief Get lane change distance with ADC
   * @param Target lane sequence
   * @return Lane change distance with ADC
   */
  double GetLaneChangeDistanceWithADC(const LaneSequence& lane_sequence);

  /**
   * @brief Clear private members
   */
  void Clear();

  /**
   * @brief Convert a lane sequence to string
   * @param Lane sequence
   * @return String describing the lane sequence
   */
  std::string ToString(const LaneSequence& sequence);

 private:
  /**
   * @brief Pick the lane sequence with highest probability
   *        STRAIGHT lane sequence precedes LEFT/RIGHT lane sequences
   * @param Lane change type
   * @param Lane sequence probability
   * @param Hightest probability
   * @return Boolean if the lane sequence is enabled
   */
  bool LaneSequenceWithMaxProb(const LaneChangeType& type,
                               const double& probability,
                               const double& max_prob);

  /**
   * @brief Pick the lane change sequence with highest probability
   *        STRAIGHT lane sequence precedes LEFT/RIGHT lane sequences
   * @param Lane change type
   * @param Lane sequence probability
   * @param Hightest probability
   * @return Boolean if the lane sequence is enabled
   */
  bool LaneChangeWithMaxProb(const LaneChangeType& type,
                             const double& probability, const double& max_prob);
};

}  // namespace prediction
}  // namespace apollo

#endif
