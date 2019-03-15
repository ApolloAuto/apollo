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

#include "modules/prediction/predictor/interaction/interaction_predictor.h"

#include <limits>

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using LatLonPolynomialPair = std::pair<std::array<double, 6>,
                                       std::array<double, 5>>;
using apollo::common::TrajectoryPoint;

void InteractionPredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  const Feature& feature = obstacle->latest_feature();

  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << "] has no lane graph.";
    return;
  }

  double smallest_cost = std::numeric_limits<double>::max();
  LatLonPolynomialPair best_trajectory_lat_lon_pair;
  for (const LaneSequence& lane_sequence :
       feature.lane().lane_graph().lane_sequence()) {
    std::vector<LatLonPolynomialPair> trajectory_lat_lon_pairs =
        SampleTrajectoryPolynomials();
    for (const auto& trajectory_lat_lon_pair : trajectory_lat_lon_pairs) {
      double cost = ComputeTrajectoryCost(trajectory_lat_lon_pair);
      if (cost < smallest_cost) {
        smallest_cost = cost;
        best_trajectory_lat_lon_pair = trajectory_lat_lon_pair;
      }
    }

    double likelihood = ComputeLikelihood(smallest_cost);
    double prior = lane_sequence.probability();
    double posterior = ComputePosterior(prior, likelihood);

    double probability_threshold = 0.5;
    if (posterior < probability_threshold) {
      continue;
    }

    std::vector<TrajectoryPoint> points;
    DrawTrajectory(*obstacle, lane_sequence,
        best_trajectory_lat_lon_pair,
        FLAGS_prediction_trajectory_time_length,
        FLAGS_prediction_trajectory_time_resolution,
        &points);
    Trajectory trajectory = GenerateTrajectory(points);
    trajectory.set_probability(posterior);
    trajectories_.push_back(std::move(trajectory));
  }
}

void InteractionPredictor::Clear() { Predictor::Clear(); }

bool InteractionPredictor::DrawTrajectory(
    const Obstacle& obstacle, const LaneSequence& lane_sequence,
    const LatLonPolynomialPair& trajectory_lat_lon_pair,
    const double total_time, const double period,
    std::vector<TrajectoryPoint>* points) {
  // TODO(kechxu) implement
  return false;
}

std::vector<LatLonPolynomialPair>
InteractionPredictor::SampleTrajectoryPolynomials() {
  std::vector<LatLonPolynomialPair> trajectory_lat_lon_pairs;
  // TODO(kechxu) implement
  return trajectory_lat_lon_pairs;
}

double InteractionPredictor::ComputeTrajectoryCost(
    const LatLonPolynomialPair& trajectory_lat_lon_pair) {
  // TODO(kechxu) implement
  // * centripetal acc
  // * collision with ego vehicle if his right of way is lower
  return 0.0;
}

double InteractionPredictor::ComputeLikelihood(const double cost) {
  // TODO(kechxu) adjust alpha
  double alpha = 1.0;
  return std::exp(-alpha * cost);
}

double InteractionPredictor::ComputePosterior(
    const double prior, const double likelihood) {
  return prior * likelihood;
}

}  // namespace prediction
}  // namespace apollo
