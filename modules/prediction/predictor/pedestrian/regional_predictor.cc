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

#include <limits>
#include <cmath>

#include "modules/prediction/predictor/pedestrian/regional_predictor.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/common/math/kalman_filter.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;
using apollo::common::PathPoint;
using apollo::common::math::KalmanFilter;

namespace {

Eigen::Vector2d GetUnitVector2d(
    const TrajectoryPoint& from_point, const TrajectoryPoint& to_point) {
  double delta_x = to_point.path_point().x() - from_point.path_point().x();
  double delta_y = to_point.path_point().y() - from_point.path_point().y();
  if (std::fabs(delta_x) <= std::numeric_limits<double>::epsilon()) {
    delta_x = 0.0;
  }
  if (std::fabs(delta_y) <= std::numeric_limits<double>::epsilon()) {
    delta_y = 0.0;
  }

  double norm = std::hypot(delta_x, delta_y);
  if (norm > 1e-10) {
  	delta_x /= norm;
  	delta_y /= norm;
  }
  return {delta_x, delta_y};
}

void CompressVector2d(const double to_length, Eigen::Vector2d* vec) {
  double norm = std::hypot(vec->operator[](0), vec->operator[](1));
  if (norm > to_length) {
  	double ratio = to_length / norm;
  	vec->operator[](0) *= ratio;
  	vec->operator[](1) *= ratio;
  }
}

}  // namespace

void RegionalPredictor::Predict(Obstacle* obstacle) {}

void RegionalPredictor::GetTrajectoryCandidatePoints(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acceleration,
    const KalmanFilter<double, 2, 2, 4>& kf,
    const double total_time,
    std::vector<TrajectoryPoint>* middle_points,
    std::vector<TrajectoryPoint>* boundary_points) {
  // TODO(kechxu) implement
}

void RegionalPredictor::UpdateTrajectoryPoints(
    const TrajectoryPoint& starting_point,
    const Eigen::Vector2d& velocity,
    const double delta_ts,
    const std::vector<TrajectoryPoint>& middle_points,
    const std::vector<TrajectoryPoint>& boundary_points,
    std::vector<TrajectoryPoint>* left_points,
    std::vector<TrajectoryPoint>* right_points) {
  if (2 * middle_points.size() != boundary_points.size()) {
    AERROR << "Middle and ellipse points sizes not match";
  }

  // double speed = std::hypot(velocity[0], velocity[1]);
  // TODO(kechxu) continue implementing
}

void RegionalPredictor::InsertTrajectoryPoint(
    const TrajectoryPoint& prev_middle_point,
    const Eigen::Vector2d& middle_direction,
    const TrajectoryPoint& boundary_point,
    const double speed,
    const double delta_ts,
    int* left_i,
    int* right_i,
    double* left_heading,
    double* right_heading,
    std::vector<TrajectoryPoint>* left_points,
    std::vector<TrajectoryPoint>* right_points) {
  // TODO(kechxu) implement
}

void RegionalPredictor::GetTwoEllipsePoints(
    const double position_x, const double position_y,
    const double direction_x, const double direction_y,
    const double ellipse_len_x, const double ellipse_len_y,
    TrajectoryPoint* ellipse_point_1,
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
  GetQuadraticCoefficients(position_x, position_y,
      direction_x, direction_y, ellipse_len_x, ellipse_len_y, &coefficients);
  std::pair<double, double> roots(position_x, position_y);
  apollo::prediction::util::SolveQuadraticEquation(coefficients, &roots);
  double temp_p = 0.0 - direction_x / direction_y;
  double temp_q = position_y - temp_p * position_x;

  double ellipse_point_1_x = roots.first;
  double ellipse_point_2_x = roots.second;

  double ellipse_point_1_y = temp_p * ellipse_point_1_x + temp_q;
  double ellipse_point_2_y = temp_p * ellipse_point_2_x + temp_q;

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
  double temp_p = 0.0 - direction_x / direction_y;
  double temp_q = position_y - temp_p * position_x;

  // three coefficients a, b, c for the equation a x^2 + b x + c = 0
  double coefficient_a = ellipse_len_y * ellipse_len_y +
                         ellipse_len_x * ellipse_len_x * temp_p * temp_p;

  double coefficient_b =
      0.0 - 2.0 * position_x * ellipse_len_y * ellipse_len_y +
      2.0 * temp_p * (temp_q - position_y) * ellipse_len_x * ellipse_len_x;

  double coefficient_c =
      ellipse_len_x * ellipse_len_x * (temp_q - position_y) *
          (temp_q - position_y) -
      ellipse_len_x * ellipse_len_x * ellipse_len_y * ellipse_len_y +
      ellipse_len_y * ellipse_len_y * position_x * position_x;

  coefficients->push_back(std::move(coefficient_a));
  coefficients->push_back(std::move(coefficient_b));
  coefficients->push_back(std::move(coefficient_c));
}

void RegionalPredictor::UpdateHeading(
    const TrajectoryPoint& curr_point,
    std::vector<TrajectoryPoint>* points) {
  if (points->empty()) {
    return;
  }
  TrajectoryPoint& prev_point = points->back();
  double delta_x = curr_point.path_point().x() - prev_point.path_point().x();
  double delta_y = curr_point.path_point().y() - prev_point.path_point().y();
  prev_point.mutable_path_point()->set_theta(std::atan2(delta_y, delta_x));
}

void RegionalPredictor::TranslatePoint(
    const double translate_x, const double translate_y,
    TrajectoryPoint* point) {
  double original_x = point->path_point().x();
  double original_y = point->path_point().y();
  point->mutable_path_point()->set_x(original_x + translate_x);
  point->mutable_path_point()->set_y(original_y + translate_y);
}

}  // namespace prediction
}  // namespace apollo
