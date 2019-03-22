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
  InteractionPredictor();

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
  class LatLonPolynomialBundle {
   public:
    LatLonPolynomialBundle() = default;
    ~LatLonPolynomialBundle() = default;
    LatLonPolynomialBundle(
        const std::array<double, 6>& lat_polynomial_coeffs,
        const std::array<double, 5>& lon_polynomial_coeffs,
        const double lat_end_t,
        const double lon_end_t,
        const double lon_end_v) :
      lat_polynomial_coeffs_(lat_polynomial_coeffs),
      lon_polynomial_coeffs_(lon_polynomial_coeffs),
      lat_end_t_(lat_end_t),
      lon_end_t_(lon_end_t),
      lon_end_v_(lon_end_v) {}

    std::array<double, 6> lat_polynomial_coeffs() const {
      return lat_polynomial_coeffs_;
    }
    std::array<double, 5> lon_polynomial_coeffs() const {
      return lon_polynomial_coeffs_;
    }
    double lat_end_t() const { return lat_end_t_; }
    double lon_end_t() const { return lon_end_t_; }
    double lon_end_v() const { return lon_end_v_; }

   private:
    std::array<double, 6> lat_polynomial_coeffs_;
    std::array<double, 5> lon_polynomial_coeffs_;
    double lat_end_t_;
    double lon_end_t_;
    double lon_end_v_;
  };

  void Clear();

  void BuildADCTrajectory(const double resolution);

  bool DrawTrajectory(
    const Obstacle& obstacle, const LaneSequence& lane_sequence,
    const LatLonPolynomialBundle& lat_lon_polynomial_bundle,
    const double total_time, const double period,
    std::vector<apollo::common::TrajectoryPoint>* trajectory_points);

  bool SampleTrajectoryPolynomials(
      const Obstacle& obstacle,
      const LaneSequence& lane_sequence,
      std::vector<LatLonPolynomialBundle>* lat_lon_polynomial_bundles);

  double ComputeTrajectoryCost(
      const Obstacle& obstacle,
      const LaneSequence& lane_sequence,
      const LatLonPolynomialBundle& lat_lon_polynomial_bundle);

  double CentripetalAccelerationCost(
      const LaneSequence& lane_sequence,
      const LatLonPolynomialBundle& lat_lon_polynomial_bundle);

  double CollisionWithEgoVehicleCost(
      const LaneSequence& lane_sequence,
      const LatLonPolynomialBundle& lat_lon_polynomial_bundle);

  bool LowerRightOfWayThanEgo(const Obstacle& obstacle);

  double ComputeLikelihood(const double cost);

  double ComputePosterior(const double prior, const double likelihood);

 private:
  std::vector<apollo::common::TrajectoryPoint> adc_trajectory_;
};

}  // namespace prediction
}  // namespace apollo
