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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/prediction/predictor/interaction/interaction_predictor.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::hdmap::LaneInfo;
using apollo::prediction::math_util::EvaluateQuarticPolynomial;
using apollo::prediction::math_util::EvaluateQuinticPolynomial;

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
  LatLonPolynomialBundle best_trajectory_lat_lon_bundle;
  for (const LaneSequence& lane_sequence :
       feature.lane().lane_graph().lane_sequence()) {
    std::vector<LatLonPolynomialBundle> trajectory_lat_lon_bundles;
    SampleTrajectoryPolynomials(*obstacle, lane_sequence,
        &trajectory_lat_lon_bundles);
    for (const auto& trajectory_lat_lon_bundle : trajectory_lat_lon_bundles) {
      double cost = ComputeTrajectoryCost(*obstacle, trajectory_lat_lon_bundle);
      if (cost < smallest_cost) {
        smallest_cost = cost;
        best_trajectory_lat_lon_bundle = trajectory_lat_lon_bundle;
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
        best_trajectory_lat_lon_bundle,
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
    const LatLonPolynomialBundle& trajectory_lat_lon_bundle,
    const double total_time, const double period,
    std::vector<TrajectoryPoint>* trajectory_points) {
  // Sanity check.
  CHECK_NOTNULL(trajectory_points);
  trajectory_points->clear();
  const Feature& feature = obstacle.latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle.id()
           << " is missing position or velocity";
    return false;
  }

  // Evaluate all candidates using the cost function and select the best one.

  // Set up some initial conditions.
  Eigen::Vector2d position(feature.position().x(), feature.position().y());

  std::array<double, 6> lateral_coeffs =
      trajectory_lat_lon_bundle.lat_polynomial_coeffs();
  std::array<double, 5> longitudinal_coeffs =
      trajectory_lat_lon_bundle.lon_polynomial_coeffs();
  double lon_end_t = trajectory_lat_lon_bundle.lon_end_t();
  double lon_end_v = trajectory_lat_lon_bundle.lon_end_v();
  double lat_end_t = trajectory_lat_lon_bundle.lat_end_t();

  int lane_segment_index = 0;
  std::string lane_id =
      lane_sequence.lane_segment(lane_segment_index).lane_id();
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById(lane_id);
  double lane_s = 0.0;
  double lane_l = 0.0;
  if (!PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l)) {
    AERROR << "Failed in getting lane s and lane l";
    return false;
  }
  double prev_lane_l = lane_l;

  // Draw each trajectory point within the total time of prediction
  size_t total_num = static_cast<size_t>(total_time / period);
  for (size_t i = 0; i < total_num; ++i) {
    double relative_time = static_cast<double>(i) * period;
    Eigen::Vector2d point;
    double theta = M_PI;

    lane_l = EvaluateQuinticPolynomial(lateral_coeffs, relative_time, 0,
                                       lat_end_t, 0.0);
    double curr_s =
        EvaluateQuarticPolynomial(longitudinal_coeffs, relative_time, 0,
                                  lon_end_t, lon_end_v);
    double prev_s = (i > 0) ? EvaluateQuarticPolynomial(
                                  longitudinal_coeffs, relative_time - period,
                                  0, lon_end_t, lon_end_v)
                            : 0.0;
    lane_s += std::max(0.0, (curr_s - prev_s));
    if (curr_s + FLAGS_double_precision < prev_s) {
      lane_l = prev_lane_l;
    }
    if (!PredictionMap::SmoothPointFromLane(lane_id, lane_s, lane_l, &point,
                                            &theta)) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << lane_l << "]";
      break;
    }

    prev_lane_l = lane_l;
    double lane_speed =
        EvaluateQuarticPolynomial(longitudinal_coeffs, relative_time, 1,
                                  lon_end_t, lon_end_v);
    double lane_acc =
        EvaluateQuarticPolynomial(longitudinal_coeffs, relative_time, 2,
                                  lon_end_t, lon_end_v);

    TrajectoryPoint trajectory_point;
    PathPoint path_point;
    path_point.set_x(point.x());
    path_point.set_y(point.y());
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    path_point.set_lane_id(lane_id);
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(lane_speed);
    trajectory_point.set_a(lane_acc);
    trajectory_point.set_relative_time(relative_time);
    trajectory_points->emplace_back(std::move(trajectory_point));

    while (lane_s > PredictionMap::LaneById(lane_id)->total_length() &&
           lane_segment_index + 1 < lane_sequence.lane_segment_size()) {
      lane_segment_index += 1;
      lane_s = lane_s - PredictionMap::LaneById(lane_id)->total_length();
      lane_id = lane_sequence.lane_segment(lane_segment_index).lane_id();
    }
  }

  return true;
}

bool InteractionPredictor::SampleTrajectoryPolynomials(
    const Obstacle& obstacle,
    const LaneSequence& lane_sequence,
    std::vector<LatLonPolynomialBundle>* lat_lon_polynomial_bundles) {
  lat_lon_polynomial_bundles->clear();
  CHECK_GT(obstacle.history_size(), 0);
  const Feature& feature = obstacle.latest_feature();
  double curr_v = feature.speed();
  std::vector<double> candidate_times{1.0, 2.0, 3.0};
  int num_v_sample = 5;
  double v_gap = curr_v / static_cast<double>(num_v_sample);
  std::vector<double> candidate_speeds;
  for (int i = 0; i < num_v_sample; ++i) {
    candidate_speeds.push_back(v_gap * static_cast<double>(i) + 1.0);
  }
  for (const double lon_end_v : candidate_speeds) {
    for (const double lon_end_t : candidate_times) {
      double lat_end_t = 3.0;
      std::array<double, 6> lat_coeffs;
      std::array<double, 5> lon_coeffs;
      GetLongitudinalPolynomial(
          obstacle, lane_sequence, {lon_end_v, lon_end_t}, &lon_coeffs);
      GetLateralPolynomial(obstacle, lane_sequence, lat_end_t, &lat_coeffs);
      lat_lon_polynomial_bundles->emplace_back(
          lat_coeffs, lon_coeffs, lat_end_t, lon_end_t, lon_end_v);
    }
  }
  return true;
}

double InteractionPredictor::ComputeTrajectoryCost(
    const Obstacle& obstacle,
    const LatLonPolynomialBundle& lat_lon_polynomial_bundle) {
  // TODO(kechxu) adjust and move to gflags
  double centri_acc_weight = 1.0;
  double collision_weight = 1.0;
  double total_cost = 0.0;
  double centri_acc_cost =
      CentripetalAccelerationCost(lat_lon_polynomial_bundle);
  total_cost += centri_acc_weight * centri_acc_cost;
  if (LowerRightOfWayThanEgo(obstacle)) {
    double collision_cost =
      CollisionWithEgoVehicleCost(lat_lon_polynomial_bundle);
    total_cost += collision_weight * collision_cost;
  }
  return total_cost;
}

double InteractionPredictor::CentripetalAccelerationCost(
    const LatLonPolynomialBundle& lat_lon_polynomial_bundle) {
  // TODO(kechxu) implement
  return 0.0;
}

double InteractionPredictor::CollisionWithEgoVehicleCost(
    const LatLonPolynomialBundle& lat_lon_polynomial_bundle) {
  // TODO(kechxu) implement
  return 0.0;
}

bool InteractionPredictor::LowerRightOfWayThanEgo(const Obstacle& obstacle) {
  // TODO(kechxu) implement
  return false;
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
