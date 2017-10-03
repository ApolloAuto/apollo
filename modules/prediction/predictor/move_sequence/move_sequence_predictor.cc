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

#include <cmath>

#include "modules/prediction/predictor/move_sequence/move_sequence_predictor.h"

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;
using apollo::common::math::KalmanFilter;

namespace {

void WeightedMean(
    const TrajectoryPoint& point1,
    const TrajectoryPoint& point2,
    const double weight1, const double weight2,
    TrajectoryPoint* ret_point) {
  // TODO(all) Double check if the following CHECK is okay.
  CHECK_EQ(point1.relative_time(), point2.relative_time());

  double ret_x = weight1 * point1.path_point().x() +
                 weight2 * point2.path_point().x();
  double ret_y = weight1 * point1.path_point().y() +
                 weight2 * point2.path_point().y();
  double ret_z = 0.0;
  double ret_theta = weight1 * point1.path_point().theta() +
                     weight2 * point2.path_point().theta();
  double ret_v = weight1 * point1.v() + weight2 * point2.v();
  double ret_a = weight1 * point1.a() + weight2 * point2.a();
  double ret_relative_time = point1.relative_time();

  ret_point->mutable_path_point()->set_x(ret_x);
  ret_point->mutable_path_point()->set_y(ret_y);
  ret_point->mutable_path_point()->set_z(ret_z);
  ret_point->mutable_path_point()->set_theta(ret_theta);
  ret_point->set_v(ret_v);
  ret_point->set_a(ret_a);
  ret_point->set_relative_time(ret_relative_time);
}

}  // namespace

void MoveSequencePredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);
}

void MoveSequencePredictor::DrawLaneSequenceTrajectoryPoints(
    const KalmanFilter<double, 4, 2, 0>& kf,
    const LaneSequence& lane_sequence,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {

  points->clear();
  std::vector<TrajectoryPoint> maneuver_trajectory_points;
  std::vector<TrajectoryPoint> motion_trajectory_points;
  DrawManeuverTrajectoryPoints(kf, lane_sequence, total_time, freq,
      &maneuver_trajectory_points);
  DrawMotionTrajectoryPoints(kf, lane_sequence, total_time, freq,
      &motion_trajectory_points);
  CHECK_EQ(maneuver_trajectory_points.size(),
           motion_trajectory_points.size());
  double t = 0.0;
  for (size_t i = 0; i < maneuver_trajectory_points.size(); ++i) {
    double motion_weight = MotionWeight(t);
    const TrajectoryPoint& maneuver_point = maneuver_trajectory_points[i];
    const TrajectoryPoint& motion_point = motion_trajectory_points[i];
    TrajectoryPoint trajectory_point;
    WeightedMean(maneuver_point, motion_point,
        1 - motion_weight, motion_weight, &trajectory_point);
    points->push_back(trajectory_point);
  }
}

void MoveSequencePredictor::DrawManeuverTrajectoryPoints(
    const KalmanFilter<double, 4, 2, 0>& kf,
    const LaneSequence& lane_sequence,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {
  // TODO(kechxu) implement
}

void MoveSequencePredictor::DrawMotionTrajectoryPoints(
    const KalmanFilter<double, 4, 2, 0>& kf,
    const LaneSequence& lane_sequence,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {
  // TODO(kechxu) implement
}

double MoveSequencePredictor::Cost(const double t,
    const std::array<double, COEFF_SIZE>& coeffs,
    const double alpha) {
  // TODO(kechxu) implement
  return 0.0;
}

double MoveSequencePredictor::MotionWeight(const double t) {
  // TODO(kechxu) Avoid the following hard-coded constants
  double a = 1.2;
  double b = 5.0;
  double c = 1.5;

  return 1.0 - 1.0 / (1.0 + a * std::exp(-b * (t - c)));
}

}  // namespace prediction
}  // namespace apollo
