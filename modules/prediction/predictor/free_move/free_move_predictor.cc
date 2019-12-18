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

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/proto/prediction_conf.pb.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;
using apollo::perception::PerceptionObstacle;

FreeMovePredictor::FreeMovePredictor() {
  predictor_type_ = ObstacleConf::FREE_MOVE_PREDICTOR;
}

bool FreeMovePredictor::Predict(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  obstacle->SetPredictorType(predictor_type_);

  const Feature& feature = obstacle->latest_feature();

  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle->id()
           << " is missing position or velocity";
    return false;
  }

  double prediction_total_time = FLAGS_prediction_trajectory_time_length;
  if (obstacle->type() == PerceptionObstacle::PEDESTRIAN) {
    prediction_total_time = FLAGS_prediction_trajectory_time_length;
  }

  if (feature.predicted_trajectory().empty()) {
    std::vector<TrajectoryPoint> points;
    Eigen::Vector2d position(feature.position().x(), feature.position().y());
    Eigen::Vector2d velocity(feature.velocity().x(), feature.velocity().y());
    Eigen::Vector2d acc(feature.acceleration().x(), feature.acceleration().y());
    double theta = feature.velocity_heading();

    DrawFreeMoveTrajectoryPoints(
        position, velocity, acc, theta, 0.0, prediction_total_time,
        FLAGS_prediction_trajectory_time_resolution, &points);

    Trajectory trajectory = GenerateTrajectory(points);
    obstacle->mutable_latest_feature()->add_predicted_trajectory()->CopyFrom(
        trajectory);
    SetEqualProbability(1.0, 0, obstacle);
  } else {
    for (int i = 0; i < feature.predicted_trajectory_size(); ++i) {
      Trajectory* trajectory =
          obstacle->mutable_latest_feature()->mutable_predicted_trajectory(i);
      int traj_size = trajectory->trajectory_point_size();
      if (traj_size == 0) {
        AERROR << "Empty predicted trajectory found";
        continue;
      }
      std::vector<TrajectoryPoint> points;
      const TrajectoryPoint& last_point =
          trajectory->trajectory_point(traj_size - 1);
      double theta = last_point.path_point().theta();
      Eigen::Vector2d position(last_point.path_point().x(),
                               last_point.path_point().y());
      Eigen::Vector2d velocity(last_point.v() * std::cos(theta),
                               last_point.v() * std::sin(theta));
      Eigen::Vector2d acc(last_point.a() * std::cos(theta),
                          last_point.a() * std::sin(theta));
      double last_relative_time = last_point.relative_time();
      DrawFreeMoveTrajectoryPoints(
          position, velocity, acc, theta, last_relative_time,
          prediction_total_time - last_relative_time,
          FLAGS_prediction_trajectory_time_resolution, &points);
      // The following for-loop starts from index 1 because the vector points
      // includes the last point in the existing predicted trajectory
      for (size_t i = 1; i < points.size(); ++i) {
        trajectory->add_trajectory_point()->CopyFrom(points[i]);
      }
    }
  }
  return true;
}

void FreeMovePredictor::DrawFreeMoveTrajectoryPoints(
    const Eigen::Vector2d& position, const Eigen::Vector2d& velocity,
    const Eigen::Vector2d& acc, const double theta, const double start_time,
    const double total_time, const double period,
    std::vector<TrajectoryPoint>* points) {
  Eigen::Matrix<double, 6, 1> state;
  state.setZero();
  state(0, 0) = 0.0;
  state(1, 0) = 0.0;
  state(2, 0) = velocity(0);
  state(3, 0) = velocity(1);
  state(4, 0) = common::math::Clamp(acc(0), FLAGS_vehicle_min_linear_acc,
                                    FLAGS_vehicle_max_linear_acc);
  state(5, 0) = common::math::Clamp(acc(1), FLAGS_vehicle_min_linear_acc,
                                    FLAGS_vehicle_max_linear_acc);

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
      &state, transition, theta, start_time, num, period, points);

  for (size_t i = 0; i < points->size(); ++i) {
    ::apollo::prediction::predictor_util::TranslatePoint(
        position[0], position[1], &(points->operator[](i)));
  }
}

}  // namespace prediction
}  // namespace apollo
