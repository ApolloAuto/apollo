/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @brief Define interaction predictor
 */

#pragma once

#include <vector>
#include <utility>

#include "modules/prediction/predictor/sequence/sequence_predictor.h"

namespace apollo {
namespace prediction {

class InteractionPredictor : public SequencePredictor {
 public:
  /**
   * @brief Constructor
   */
  InteractionPredictor() = default;

  /**
   * @brief Destructor
   */
  virtual ~InteractionPredictor() = default;

  /**
   * @brief Make prediction
   * @param Obstacle pointer
   */
  void Predict(Obstacle* obstacle) override;

 private:
  struct LatLonPolynomialBundle {
    std::array<double, 6> lat_polynomial_coeffs;
    std::array<double, 5> lon_polynomial_coeffs;
    double end_t;
    double end_v;
  };

  void Clear();

  bool DrawTrajectory(
    const Obstacle& obstacle, const LaneSequence& lane_sequence,
    const LatLonPolynomialBundle& lat_lon_polynomial_bundle,
    const double total_time, const double period,
    std::vector<apollo::common::TrajectoryPoint>* trajectory_points);

  bool SampleTrajectoryPolynomials(
      std::vector<LatLonPolynomialBundle>* lat_lon_polynomial_bundles);

  double ComputeTrajectoryCost(
      const LatLonPolynomialBundle& lat_lon_polynomial_bundle);

  double ComputeLikelihood(const double cost);

  double ComputePosterior(const double prior, const double likelihood);
};

}  // namespace prediction
}  // namespace apollo
