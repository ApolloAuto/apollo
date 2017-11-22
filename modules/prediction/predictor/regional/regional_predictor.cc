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

#include "modules/prediction/predictor/regional/regional_predictor.h"

#include <cmath>
#include <limits>
#include <utility>

#include "modules/common/math/kalman_filter.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::KalmanFilter;

namespace {

Eigen::Vector2d GetUnitVector2d(const TrajectoryPoint& from_point,
                                const TrajectoryPoint& to_point) {
  double delta_x = to_point.path_point().x() - from_point.path_point().x();
  double delta_y = to_point.path_point().y() - from_point.path_point().y();
  if (std::fabs(delta_x) <= std::numeric_limits<double>::epsilon()) {
    delta_x = 0.0;
  }
  if (std::fabs(delta_y) <= std::numeric_limits<double>::epsilon()) {
    delta_y = 0.0;
  }

  const double norm = std::hypot(delta_x, delta_y);
  if (norm > 1e-10) {
    delta_x /= norm;
    delta_y /= norm;
  }
  return {delta_x, delta_y};
}

void CompressVector2d(const double to_length, Eigen::Vector2d* vec) {
  const double norm = std::hypot(vec->operator[](0), vec->operator[](1));
  if (norm > to_length) {
    const double ratio = to_length / norm;
    vec->operator[](0) *= ratio;
    vec->operator[](1) *= ratio;
  }
}

double CrossProduct(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
  return vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

}  // namespace

void RegionalPredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  double speed = 0.0;
  const Feature& feature = obstacle->latest_feature();
  if (feature.has_speed()) {
    speed = feature.speed();
  }
  if (FLAGS_enable_kf_tracking && feature.has_t_speed()) {
    speed = feature.t_speed();
  }
  if (speed > FLAGS_still_speed) {
    GenerateMovingTrajectory(obstacle, 1.0);
  } else {
    GenerateStillTrajectory(obstacle, 1.0);
  }
}

void RegionalPredictor::GenerateStillTrajectory(const Obstacle* obstacle,
                                                double probability) {
  if (obstacle == nullptr) {
    AERROR << "Missing obstacle.";
    return;
  }
  const Feature& feature = obstacle->latest_feature();
  if (!feature.has_position() || !feature.position().has_x() ||
      !feature.position().has_y() || !feature.has_velocity()) {
    AERROR << "Missing position or velocity.";
    return;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  double heading = 0.0 - M_PI;
  const int num_traj = FLAGS_num_trajectory_still_pedestrian;
  const double delta_heading = 2.0 * M_PI / num_traj;
  const double speed = FLAGS_pedestrian_min_speed;
  const double total_time = FLAGS_prediction_pedestrian_total_time;
  const int start_index = NumOfTrajectories();

  for (int i = 0; i < num_traj; ++i) {
    std::vector<TrajectoryPoint> points;
    DrawStillTrajectory(position, heading, speed, total_time, &points);
    Trajectory trajectory = GenerateTrajectory(points);
    trajectories_.push_back(std::move(trajectory));
    heading += delta_heading;
  }
  SetEqualProbability(probability, start_index);
}

void RegionalPredictor::GenerateMovingTrajectory(const Obstacle* obstacle,
                                                 double probability) {
  if (obstacle == nullptr) {
    AERROR << "Missing obstacle.";
    return;
  }
  const Feature& feature = obstacle->latest_feature();
  if (!feature.has_position() || !feature.position().has_x() ||
      !feature.position().has_y() || !feature.has_velocity()) {
    AERROR << "Missing position or velocity.";
    return;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  Eigen::Vector2d velocity(feature.velocity().x(), feature.velocity().y());
  Eigen::Vector2d acc(0.0, 0.0);
  if (FLAGS_enable_kf_tracking) {
    velocity[0] = feature.t_velocity().x();
    velocity[1] = feature.t_velocity().y();
  }
  if (FLAGS_enable_pedestrian_acc) {
    acc = {feature.acceleration().x(), feature.acceleration().y()};
    if (FLAGS_enable_kf_tracking) {
      acc = {feature.t_acceleration().x(), feature.t_acceleration().y()};
    }
  }

  const double total_time = FLAGS_prediction_pedestrian_total_time;
  std::vector<TrajectoryPoint> left_points;
  std::vector<TrajectoryPoint> right_points;

  DrawMovingTrajectory(position, velocity, acc,
                       obstacle->kf_pedestrian_tracker(), total_time,
                       &left_points, &right_points);
  int start_index = NumOfTrajectories();

  Trajectory left_trajectory = GenerateTrajectory(left_points);
  Trajectory right_trajectory = GenerateTrajectory(right_points);
  trajectories_.push_back(std::move(left_trajectory));
  trajectories_.push_back(std::move(right_trajectory));
  SetEqualProbability(probability, start_index);
}

void RegionalPredictor::DrawStillTrajectory(
    const Eigen::Vector2d& position, const double heading, const double speed,
    const double total_time, std::vector<TrajectoryPoint>* points) {
  double delta_ts = FLAGS_prediction_freq;
  double x = position[0];
  double y = position[1];
  double direction_x = std::cos(heading);
  double direction_y = std::sin(heading);
  for (int i = 0; i < static_cast<int>(total_time / delta_ts); ++i) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(x);
    point.mutable_path_point()->set_y(y);
    point.mutable_path_point()->set_theta(heading);
    point.set_v(speed);
    point.set_relative_time(i * delta_ts);
    points->push_back(std::move(point));
    x += direction_x * speed * delta_ts;
    y += direction_y * speed * delta_ts;
  }
}

void RegionalPredictor::DrawMovingTrajectory(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acceleration,
    const apollo::common::math::KalmanFilter<double, 2, 2, 4>& kf,
    const double total_time, std::vector<TrajectoryPoint>* left_points,
    std::vector<TrajectoryPoint>* right_points) {
  double delta_ts = FLAGS_prediction_freq;
  Eigen::Vector2d vel = velocity;
  CompressVector2d(FLAGS_pedestrian_max_speed, &vel);
  Eigen::Vector2d acc = acceleration;
  CompressVector2d(FLAGS_pedestrian_max_acc, &acc);
  double speed = std::hypot(vel[0], vel[1]);

  // candidate point sequences
  std::vector<TrajectoryPoint> middle_points;
  std::vector<TrajectoryPoint> boundary_points;

  TrajectoryPoint starting_point;
  starting_point.mutable_path_point()->set_x(0.0);
  starting_point.mutable_path_point()->set_y(0.0);
  starting_point.set_v(speed);

  Eigen::Vector2d translated_vec(0.0, 0.0);
  GetTrajectoryCandidatePoints(translated_vec, vel, acc, kf, total_time,
                               &middle_points, &boundary_points);

  if (middle_points.empty() || boundary_points.size() < 2) {
    ADEBUG << "No valid points found.";
    return;
  }

  UpdateTrajectoryPoints(starting_point, vel, delta_ts, middle_points,
                         boundary_points, left_points, right_points);
  for (size_t i = 0; i < left_points->size(); ++i) {
    apollo::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(left_points->operator[](i)));
    apollo::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(right_points->operator[](i)));
  }
}

void RegionalPredictor::GetTrajectoryCandidatePoints(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acceleration,
    const KalmanFilter<double, 2, 2, 4>& kf_pedestrian_tracker,
    const double total_time, std::vector<TrajectoryPoint>* middle_points,
    std::vector<TrajectoryPoint>* boundary_points) {
  double delta_ts = FLAGS_prediction_freq;
  KalmanFilter<double, 2, 2, 4> kf = kf_pedestrian_tracker;
  // set the control matrix and control vector
  Eigen::Matrix<double, 2, 2> P = kf.GetStateCovariance();
  Eigen::Matrix<double, 2, 1> x;
  x.setZero();
  kf.SetStateEstimate(x, P);

  Eigen::Matrix<double, 2, 4> B;
  B.setZero();
  B(0, 0) = delta_ts;
  B(0, 2) = 0.5 * delta_ts * delta_ts;
  B(1, 1) = delta_ts;
  B(1, 3) = 0.5 * delta_ts * delta_ts;
  Eigen::Matrix<double, 4, 1> u;
  u.setZero();
  u(0, 0) = velocity.x();
  u(1, 0) = velocity.y();
  if (FLAGS_enable_pedestrian_acc) {
    u(2, 0) = acceleration.x();
    u(3, 0) = acceleration.y();
  }

  kf.SetControlMatrix(B);

  TrajectoryPoint prev_middle_point;
  prev_middle_point.mutable_path_point()->set_x(position[0]);
  prev_middle_point.mutable_path_point()->set_y(position[1]);

  for (int i = 0; i < static_cast<int>(total_time / delta_ts); ++i) {
    kf.Predict(u);
    Eigen::Matrix<double, 2, 2> P = kf.GetStateCovariance();
    double ellipse_len_x = std::sqrt(std::fabs(P(0, 0)));
    double ellipse_len_y = std::sqrt(std::fabs(P(1, 1)));
    ellipse_len_x *= FLAGS_coeff_mul_sigma;
    ellipse_len_y *= FLAGS_coeff_mul_sigma;

    Eigen::Matrix<double, 2, 1> state = kf.GetStateEstimate();
    double middle_point_x = state(0, 0);
    double middle_point_y = state(1, 0);
    TrajectoryPoint middle_point;
    middle_point.mutable_path_point()->set_x(middle_point_x);
    middle_point.mutable_path_point()->set_y(middle_point_y);
    TrajectoryPoint boundary_point_1;
    TrajectoryPoint boundary_point_2;
    Eigen::Vector2d direction =
        GetUnitVector2d(prev_middle_point, middle_point);
    GetTwoEllipsePoints(middle_point_x, middle_point_y, direction[0],
                        direction[1], ellipse_len_x, ellipse_len_y,
                        &boundary_point_1, &boundary_point_2);
    prev_middle_point = middle_point;

    middle_points->push_back(std::move(middle_point));
    boundary_points->push_back(std::move(boundary_point_1));
    boundary_points->push_back(std::move(boundary_point_2));
  }
}

void RegionalPredictor::UpdateTrajectoryPoints(
    const TrajectoryPoint& starting_point, const Eigen::Vector2d& velocity,
    const double delta_ts, const std::vector<TrajectoryPoint>& middle_points,
    const std::vector<TrajectoryPoint>& boundary_points,
    std::vector<TrajectoryPoint>* left_points,
    std::vector<TrajectoryPoint>* right_points) {
  if (2 * middle_points.size() != boundary_points.size()) {
    AWARN << "Middle and ellipse points sizes not match";
  }
  double speed = std::hypot(velocity[0], velocity[1]);
  double left_heading = std::atan2(velocity[1], velocity[0]);
  double right_heading = std::atan2(velocity[1], velocity[0]);

  TrajectoryPoint left_starting_point = starting_point;
  left_points->push_back(std::move(left_starting_point));
  TrajectoryPoint right_starting_point = starting_point;
  right_points->push_back(std::move(right_starting_point));

  int left_i = 0;
  int right_i = 0;
  for (size_t i = 0; i < middle_points.size(); ++i) {
    TrajectoryPoint prev_middle_point = starting_point;
    if (i > 0) {
      prev_middle_point = middle_points[i - 1];
    }
    Eigen::Vector2d middle_direction =
        GetUnitVector2d(prev_middle_point, middle_points[i]);
    if (2 * i > boundary_points.size()) {
      break;
    }
    TrajectoryPoint boundary_point_1 = boundary_points[2 * i];
    InsertTrajectoryPoint(prev_middle_point, middle_direction, boundary_point_1,
                          speed, delta_ts, &left_i, &right_i, &left_heading,
                          &right_heading, left_points, right_points);
    if (2 * i + 1 >= boundary_points.size()) {
      break;
    }
    TrajectoryPoint boundary_point_2 = boundary_points[2 * i + 1];
    InsertTrajectoryPoint(prev_middle_point, middle_direction, boundary_point_2,
                          speed, delta_ts, &left_i, &right_i, &left_heading,
                          &right_heading, left_points, right_points);
  }

  left_points->back().set_v(speed);
  left_points->back().set_relative_time(left_i * delta_ts);
  left_points->back().mutable_path_point()->set_theta(left_heading);

  right_points->back().set_v(speed);
  right_points->back().set_relative_time(right_i * delta_ts);
  right_points->back().mutable_path_point()->set_theta(right_heading);
}

void RegionalPredictor::InsertTrajectoryPoint(
    const TrajectoryPoint& prev_middle_point,
    const Eigen::Vector2d& middle_direction,
    const TrajectoryPoint& boundary_point, const double speed,
    const double delta_ts, int* left_i, int* right_i, double* left_heading,
    double* right_heading, std::vector<TrajectoryPoint>* left_points,
    std::vector<TrajectoryPoint>* right_points) {
  Eigen::Vector2d boundary_direction =
      GetUnitVector2d(prev_middle_point, boundary_point);
  double cross_product = CrossProduct(boundary_direction, middle_direction);
  if (cross_product < 0.0) {
    if (!left_points->empty()) {
      TrajectoryPoint& prev_point = left_points->back();
      Eigen::Vector2d dir = GetUnitVector2d(prev_point, boundary_point);
      *left_heading = std::atan2(dir[1], dir[0]);
      prev_point.mutable_path_point()->set_theta(*left_heading);
      prev_point.set_v(speed);
      prev_point.set_relative_time((*left_i) * delta_ts);
      ++(*left_i);
    }
    left_points->push_back(boundary_point);
  } else {
    if (!right_points->empty()) {
      TrajectoryPoint& prev_point = right_points->back();
      Eigen::Vector2d dir = GetUnitVector2d(prev_point, boundary_point);
      *right_heading = std::atan2(dir[1], dir[0]);
      prev_point.mutable_path_point()->set_theta(*right_heading);
      prev_point.set_v(speed);
      prev_point.set_relative_time((*right_i) * delta_ts);
      ++(*right_i);
    }
    right_points->push_back(boundary_point);
  }
}

void RegionalPredictor::GetTwoEllipsePoints(
    const double position_x, const double position_y, const double direction_x,
    const double direction_y, const double ellipse_len_x,
    const double ellipse_len_y, TrajectoryPoint* ellipse_point_1,
    TrajectoryPoint* ellipse_point_2) {
  // vertical case
  if (std::fabs(direction_x) <= std::numeric_limits<double>::epsilon()) {
    ellipse_point_1->mutable_path_point()->set_x(position_x - ellipse_len_x);
    ellipse_point_1->mutable_path_point()->set_y(position_y);
    ellipse_point_2->mutable_path_point()->set_x(position_x + ellipse_len_x);
    ellipse_point_2->mutable_path_point()->set_y(position_y);
    return;
  }
  // horizontal case
  if (std::fabs(direction_y) <= std::numeric_limits<double>::epsilon()) {
    ellipse_point_1->mutable_path_point()->set_x(position_x);
    ellipse_point_1->mutable_path_point()->set_y(position_y + ellipse_len_y);
    ellipse_point_2->mutable_path_point()->set_x(position_x);
    ellipse_point_2->mutable_path_point()->set_y(position_y - ellipse_len_y);
    return;
  }
  // general case
  std::vector<double> coefficients;
  GetQuadraticCoefficients(position_x, position_y, direction_x, direction_y,
                           ellipse_len_x, ellipse_len_y, &coefficients);
  std::pair<double, double> roots(position_x, position_y);
  apollo::prediction::math_util::SolveQuadraticEquation(coefficients, &roots);
  const double temp_p = 0.0 - direction_x / direction_y;
  const double temp_q = position_y - temp_p * position_x;

  const double ellipse_point_1_x = roots.first;
  const double ellipse_point_2_x = roots.second;

  const double ellipse_point_1_y = temp_p * ellipse_point_1_x + temp_q;
  const double ellipse_point_2_y = temp_p * ellipse_point_2_x + temp_q;

  ellipse_point_1->mutable_path_point()->set_x(ellipse_point_1_x);
  ellipse_point_1->mutable_path_point()->set_y(ellipse_point_1_y);
  ellipse_point_2->mutable_path_point()->set_x(ellipse_point_2_x);
  ellipse_point_2->mutable_path_point()->set_y(ellipse_point_2_y);
}

void RegionalPredictor::GetQuadraticCoefficients(
    const double position_x, const double position_y, const double direction_x,
    const double direction_y, const double ellipse_len_x,
    const double ellipse_len_y, std::vector<double>* coefficients) {
  coefficients->clear();
  const double temp_p = 0.0 - direction_x / direction_y;
  const double temp_q = position_y - temp_p * position_x;

  // three coefficients a, b, c for the equation a x^2 + b x + c = 0
  const double coefficient_a = ellipse_len_y * ellipse_len_y +
                               ellipse_len_x * ellipse_len_x * temp_p * temp_p;

  const double coefficient_b =
      0.0 - 2.0 * position_x * ellipse_len_y * ellipse_len_y +
      2.0 * temp_p * (temp_q - position_y) * ellipse_len_x * ellipse_len_x;

  const double coefficient_c =
      ellipse_len_x * ellipse_len_x * (temp_q - position_y) *
          (temp_q - position_y) -
      ellipse_len_x * ellipse_len_x * ellipse_len_y * ellipse_len_y +
      ellipse_len_y * ellipse_len_y * position_x * position_x;

  coefficients->push_back(std::move(coefficient_a));
  coefficients->push_back(std::move(coefficient_b));
  coefficients->push_back(std::move(coefficient_c));
}

void RegionalPredictor::UpdateHeading(const TrajectoryPoint& curr_point,
                                      std::vector<TrajectoryPoint>* points) {
  if (points->empty()) {
    return;
  }
  TrajectoryPoint& prev_point = points->back();
  const double delta_x =
      curr_point.path_point().x() - prev_point.path_point().x();
  const double delta_y =
      curr_point.path_point().y() - prev_point.path_point().y();
  prev_point.mutable_path_point()->set_theta(std::atan2(delta_y, delta_x));
}

}  // namespace prediction
}  // namespace apollo
