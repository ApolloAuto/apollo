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

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::KalmanFilter;

void FreeMovePredictor::Predict(Obstacle* obstacle) {
  trajectories_.clear();
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
  if (FLAGS_enable_kf_tracking) {
    position(0) = feature.t_position().x();
    position(1) = feature.t_position().y();
    velocity(0) = feature.t_velocity().x();
    velocity(1) = feature.t_velocity().y();
    acc(0) = feature.t_acceleration().x();
    acc(1) = feature.t_acceleration().y();
  }

  std::vector<TrajectoryPoint> points(0);
  DrawFreeMoveTrajectoryPoints(
      position, velocity, acc, obstacle->kf_motion_tracker(),
      FLAGS_prediction_duration, FLAGS_prediction_freq, &points);

  Trajectory trajectory = GenerateTrajectory(points);
  int start_index = 0;
  trajectories_.push_back(std::move(trajectory));
  SetEqualProbability(1.0, start_index);
  ADEBUG << "Obstacle [" << obstacle->id() << "] has " << trajectories_.size()
         << " trajectories.";
}

void FreeMovePredictor::DrawFreeMoveTrajectoryPoints(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acc, const KalmanFilter<double, 6, 2, 0>& kf,
    double total_time, double freq, std::vector<TrajectoryPoint>* points) {
  double theta = std::atan2(velocity(1), velocity(0));

  Eigen::Matrix<double, 6, 1> state(kf.GetStateEstimate());
  state(0, 0) = 0.0;
  state(1, 0) = 0.0;
  state(2, 0) = velocity(0);
  state(3, 0) = velocity(1);
  state(4, 0) = common::math::Clamp(acc(0), FLAGS_min_acc, FLAGS_max_acc);
  state(5, 0) = common::math::Clamp(acc(1), FLAGS_min_acc, FLAGS_max_acc);

  Eigen::Matrix<double, 6, 6> transition(kf.GetTransitionMatrix());
  transition(0, 2) = freq;
  transition(0, 4) = 0.5 * freq * freq;
  transition(1, 3) = freq;
  transition(1, 5) = 0.5 * freq * freq;
  transition(2, 4) = freq;
  transition(3, 5) = freq;

  double x = state(0, 0);
  double y = state(1, 0);
  double v_x = state(2, 0);
  double v_y = state(3, 0);
  double acc_x = state(4, 0);
  double acc_y = state(5, 0);
  for (size_t i = 0; i < static_cast<size_t>(total_time / freq); ++i) {
    double speed = std::hypot(v_x, v_y);
    if (speed <= std::numeric_limits<double>::epsilon()) {
      speed = 0.0;
      v_x = 0.0;
      v_y = 0.0;
      acc_x = 0.0;
      acc_y = 0.0;
    } else if (speed > FLAGS_max_speed) {
      speed = FLAGS_max_speed;
    }

    // update theta
    if (speed > std::numeric_limits<double>::epsilon()) {
      if (points->size() > 0) {
        PathPoint* prev_point = points->back().mutable_path_point();
        theta = std::atan2(y - prev_point->y(), x - prev_point->x());
        prev_point->set_theta(theta);
      }
    } else {
      if (points->size() > 0) {
        theta = points->back().path_point().theta();
      }
    }

    // update velocity and acc
    state(2, 0) = v_x;
    state(3, 0) = v_y;
    state(4, 0) = acc_x;
    state(5, 0) = acc_y;

    // update position
    x = state(0, 0);
    y = state(1, 0);

    // Generate trajectory point
    TrajectoryPoint trajectory_point;
    PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed);
    trajectory_point.set_a(std::hypot(acc_x, acc_y));
    trajectory_point.set_relative_time(static_cast<double>(i) * freq);
    points->emplace_back(std::move(trajectory_point));

    // Update position, velocity and acceleration
    state = transition * state;
    x = state(0, 0);
    y = state(1, 0);
    v_x = state(2, 0);
    v_y = state(3, 0);
    acc_x = state(4, 0);
    acc_y = state(5, 0);
  }

  for (size_t i = 0; i < points->size(); ++i) {
    apollo::prediction::util::TranslatePoint(position[0], position[1],
                                             &(points->operator[](i)));
  }
}

}  // namespace prediction
}  // namespace apollo
