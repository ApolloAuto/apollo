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

#include "modules/prediction/predictor/free_move/free_move_predictor.h"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include "Eigen/Dense"

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

using ::apollo::common::PathPoint;
using ::apollo::common::TrajectoryPoint;
using ::apollo::common::math::KalmanFilter;
using ::apollo::perception::PerceptionObstacle;

void FreeMovePredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  const Feature& feature = obstacle->latest_feature();

  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle->id()
           << " is missing position or velocity";
    return;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  Eigen::Vector2d velocity(feature.velocity().x(), feature.velocity().y());
  Eigen::Vector2d acc(feature.acceleration().x(), feature.acceleration().y());
  double theta = feature.velocity_heading();

  std::vector<TrajectoryPoint> points(0);
  double prediction_total_time = FLAGS_prediction_duration;
  if (obstacle->type() == PerceptionObstacle::PEDESTRIAN) {
    prediction_total_time = FLAGS_prediction_pedestrian_total_time;
  }
  DrawFreeMoveTrajectoryPoints(position, velocity, acc, theta,
      prediction_total_time, FLAGS_prediction_period, &points);

  Trajectory trajectory = GenerateTrajectory(points);
  int start_index = 0;
  trajectories_.push_back(std::move(trajectory));
  SetEqualProbability(1.0, start_index);
  ADEBUG << "Obstacle [" << obstacle->id() << "] has " << trajectories_.size()
         << " trajectories.";
}

void FreeMovePredictor::DrawFreeMoveTrajectoryPoints(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acc, const double theta,
    const double total_time, const double period,
    std::vector<TrajectoryPoint>* points) {
  Eigen::Matrix<double, 6, 1> state;
  state.setZero();
  state(0, 0) = 0.0;
  state(1, 0) = 0.0;
  state(2, 0) = velocity(0);
  state(3, 0) = velocity(1);
  state(4, 0) = common::math::Clamp(acc(0), FLAGS_min_acc, FLAGS_max_acc);
  state(5, 0) = common::math::Clamp(acc(1), FLAGS_min_acc, FLAGS_max_acc);

  Eigen::Matrix<double, 6, 6> transition;
  transition.setIdentity();
  transition(0, 2) = period;
  transition(0, 4) = 0.5 * period * period;
  transition(1, 3) = period;
  transition(1, 5) = 0.5 * period * period;
  transition(2, 4) = period;
  transition(3, 5) = period;

  size_t num = static_cast<size_t>(total_time / period);
  ::apollo::prediction::predictor_util::GenerateFreeMoveTrajectoryPoints(
      &state, transition, theta, num, period, points);

  for (size_t i = 0; i < points->size(); ++i) {
    ::apollo::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(points->operator[](i)));
  }
}

}  // namespace prediction
}  // namespace apollo
