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
#include <utility>
#include <limits>

#include "Eigen/Dense"

#include "modules/prediction/predictor/move_sequence/move_sequence_predictor.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
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

  const Feature& feature = obstacle->latest_feature();
  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << " has no lane graph.";
    return;
  }

  std::string lane_id = "";
  if (feature.lane().has_lane_feature()) {
    lane_id = feature.lane().lane_feature().lane_id();
  }
  int num_lane_sequence = feature.lane().lane_graph().lane_sequence_size();
  std::vector<bool> enable_lane_sequence(num_lane_sequence, true);
  // FilterLaneSequences(feature.lane().lane_graph(), lane_id,
  //                     &enable_lane_sequence);
  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = feature.lane().lane_graph().lane_sequence(i);
    if (sequence.lane_segment_size() <= 0) {
      AERROR << "Empty lane segments.";
      continue;
    }

    if (!enable_lane_sequence[i]) {
      ADEBUG << "Lane sequence [" << ToString(sequence)
             << "] with probability [" << sequence.probability()
             << "] is disqualified.";
      continue;
    }

    ADEBUG << "Obstacle [" << obstacle->id()
           << "] will draw a lane sequence trajectory [" << ToString(sequence)
           << "] with probability [" << sequence.probability() << "].";

    std::string curr_lane_id = sequence.lane_segment(0).lane_id();
    std::vector<TrajectoryPoint> points;
    DrawLaneSequenceTrajectoryPoints(*obstacle, sequence,
        FLAGS_prediction_duration, FLAGS_prediction_freq, &points);

    Trajectory trajectory = GenerateTrajectory(points);
    trajectory.set_probability(sequence.probability());
    trajectories_.push_back(std::move(trajectory));
  }
  ADEBUG << "Obstacle [" << obstacle->id() << "] has total "
         << trajectories_.size() << " trajectories.";
}

void MoveSequencePredictor::DrawLaneSequenceTrajectoryPoints(
    const Obstacle& obstacle,
    const LaneSequence& lane_sequence,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {

  points->clear();
  std::vector<TrajectoryPoint> maneuver_trajectory_points;
  std::vector<TrajectoryPoint> motion_trajectory_points;
  DrawManeuverTrajectoryPoints(obstacle, lane_sequence, total_time, freq,
      &maneuver_trajectory_points);
  DrawMotionTrajectoryPoints(obstacle, total_time, freq,
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
    t += freq;
  }
}

void MoveSequencePredictor::DrawManeuverTrajectoryPoints(
    const Obstacle& obstacle,
    const LaneSequence& lane_sequence,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {
  // TODO(kechxu) implement
}

void MoveSequencePredictor::DrawMotionTrajectoryPoints(
    const Obstacle& obstacle,
    const double total_time, const double freq,
    std::vector<TrajectoryPoint>* points) {

  // Apply free_move here
  const Feature& feature = obstacle.latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle.id()
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
  const KalmanFilter<double, 6, 2, 0>& kf = obstacle.kf_motion_tracker();
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

std::string MoveSequencePredictor::ToString(const LaneSequence& sequence) {
  std::string str_lane_sequence = "";
  if (sequence.lane_segment_size() > 0) {
    str_lane_sequence += sequence.lane_segment(0).lane_id();
  }
  for (int i = 1; i < sequence.lane_segment_size(); ++i) {
    str_lane_sequence += ("->" + sequence.lane_segment(i).lane_id());
  }
  return str_lane_sequence;
}

}  // namespace prediction
}  // namespace apollo
