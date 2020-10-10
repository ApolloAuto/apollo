/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/predictor/junction/junction_predictor.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::hdmap::LaneInfo;
using apollo::prediction::math_util::ComputePolynomial;
using apollo::prediction::math_util::EvaluateCubicPolynomial;

JunctionPredictor::JunctionPredictor() {
  predictor_type_ = ObstacleConf::JUNCTION_PREDICTOR;
}

bool JunctionPredictor::Predict(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  Clear();
  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  obstacle->SetPredictorType(predictor_type_);

  const Feature& latest_feature = obstacle->latest_feature();
  std::vector<JunctionExit> junction_exits =
      MostLikelyJunctions(latest_feature);
  for (const auto& junction_exit : junction_exits) {
    std::vector<TrajectoryPoint> trajectory_points;
    DrawJunctionTrajectoryPoints(
        latest_feature, junction_exit, FLAGS_prediction_trajectory_time_length,
        FLAGS_prediction_trajectory_time_resolution, &trajectory_points);
    Trajectory trajectory = GenerateTrajectory(trajectory_points);
    obstacle->mutable_latest_feature()->add_predicted_trajectory()->CopyFrom(
        trajectory);
  }
  return true;
}

void JunctionPredictor::DrawJunctionTrajectoryPoints(
    const Feature& feature, const JunctionExit& junction_exit,
    const double total_time, const double period,
    std::vector<TrajectoryPoint>* trajectory_points) {
  double speed = feature.speed();
  const std::array<double, 2> start_x = {feature.position().x(),
                                         feature.raw_velocity().x()};
  const std::array<double, 2> end_x = {
      junction_exit.exit_position().x(),
      std::cos(junction_exit.exit_heading()) * speed};
  const std::array<double, 2> start_y = {feature.position().y(),
                                         feature.raw_velocity().y()};
  const std::array<double, 2> end_y = {
      junction_exit.exit_position().y(),
      std::sin(junction_exit.exit_heading()) * speed};
  double exit_time = GetBestTime(start_x, end_x, start_y, end_y);
  std::array<double, 4> x_coeffs =
      ComputePolynomial<3>(start_x, end_x, exit_time);
  std::array<double, 4> y_coeffs =
      ComputePolynomial<3>(start_y, end_y, exit_time);
  double t = 0.0;
  // Trajectory in junction
  while (t <= exit_time) {
    PathPoint path_point;
    path_point.set_x(EvaluateCubicPolynomial(x_coeffs, t, 0));
    path_point.set_y(EvaluateCubicPolynomial(y_coeffs, t, 0));
    path_point.set_z(0.0);
    path_point.set_theta(std::atan2(EvaluateCubicPolynomial(y_coeffs, t, 1),
                                    EvaluateCubicPolynomial(x_coeffs, t, 1)));
    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(std::hypot(EvaluateCubicPolynomial(x_coeffs, t, 1),
                                      EvaluateCubicPolynomial(y_coeffs, t, 1)));
    trajectory_point.set_relative_time(t);
    trajectory_points->emplace_back(std::move(trajectory_point));
    t += period;
  }
  std::string lane_id = junction_exit.exit_lane_id();
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById(lane_id);
  Eigen::Vector2d pos(junction_exit.exit_position().x(),
                      junction_exit.exit_position().y());
  double lane_s = 0.0;
  double lane_l = 0.0;
  double theta = M_PI;
  PredictionMap::GetProjection(pos, lane_info, &lane_s, &lane_l);
  // Trajectory after junction
  while (t <= total_time) {
    if (!PredictionMap::SmoothPointFromLane(lane_id, lane_s, 0.0, &pos,
                                            &theta)) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << 0.0 << "]";
      return;
    }
    TrajectoryPoint trajectory_point;
    PathPoint path_point;
    path_point.set_x(pos.x());
    path_point.set_y(pos.y());
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    path_point.set_lane_id(lane_id);
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed);
    trajectory_point.set_a(0.0);
    trajectory_point.set_relative_time(t);
    trajectory_points->emplace_back(std::move(trajectory_point));
    lane_s += speed * period;
    while (lane_s > PredictionMap::LaneById(lane_id)->total_length()) {
      lane_s = lane_s - PredictionMap::LaneById(lane_id)->total_length();
      if (PredictionMap::LaneById(lane_id)->lane().successor_id_size() < 1) {
        return;
      }
      // TODO(all) consider the logic to choose successor_id
      lane_id = PredictionMap::LaneById(lane_id)->lane().successor_id(0).id();
    }
    t += period;
  }
}

std::vector<JunctionExit> JunctionPredictor::MostLikelyJunctions(
    const Feature& feature) {
  if (!feature.has_junction_feature()) {
    AERROR << "No junction_feature exist!";
    return {};
  }
  if (feature.junction_feature().junction_exit_size() < 1 ||
      feature.junction_feature().junction_mlp_probability_size() != 12) {
    AERROR << "No junction_exit"
           << "or no enough junction_mlp_probability to process!";
    return {};
  }
  int max_idx = 0;
  double max_prob = 0.0;
  for (int i = 0; i < 12; ++i) {
    if (feature.junction_feature().junction_mlp_probability(i) > max_prob) {
      max_prob = feature.junction_feature().junction_mlp_probability(i);
      max_idx = i;
    }
  }
  std::vector<JunctionExit> junction_exits;
  for (const JunctionExit& junction_exit :
       feature.junction_feature().junction_exit()) {
    double x = junction_exit.exit_position().x() - feature.position().x();
    double y = junction_exit.exit_position().y() - feature.position().y();
    double angle = std::atan2(y, x) - std::atan2(feature.raw_velocity().y(),
                                                 feature.raw_velocity().x());
    double d_idx = (angle / (2.0 * M_PI)) * 12.0;
    int idx = static_cast<int>(floor(d_idx >= 0 ? d_idx : d_idx + 12));
    if (idx == max_idx) {
      junction_exits.push_back(junction_exit);
    }
  }
  return junction_exits;
}

double JunctionPredictor::GetBestTime(const std::array<double, 2>& start_x,
                                      const std::array<double, 2>& end_x,
                                      const std::array<double, 2>& start_y,
                                      const std::array<double, 2>& end_y) {
  // Generate candidate finish times.
  std::vector<double> candidate_times = GenerateCandidateTimes();
  double best_time = 0.0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < candidate_times.size(); ++i) {
    double time_to_exit = candidate_times[i];
    std::array<double, 4> x_coeffs =
        ComputePolynomial<3>(start_x, end_x, time_to_exit);
    std::array<double, 4> y_coeffs =
        ComputePolynomial<3>(start_y, end_y, time_to_exit);
    double cost_of_trajectory = CostFunction(x_coeffs, y_coeffs, time_to_exit);
    if (cost_of_trajectory <= best_cost) {
      best_cost = cost_of_trajectory;
      best_time = time_to_exit;
    }
  }
  return best_time;
}

double JunctionPredictor::CostFunction(const std::array<double, 4>& x_coeffs,
                                       const std::array<double, 4>& y_coeffs,
                                       const double time_to_exit) {
  double t = 0.0;
  double cost = 0.0;
  while (t <= time_to_exit) {
    double x_1 = EvaluateCubicPolynomial(x_coeffs, t, 1);
    double x_2 = EvaluateCubicPolynomial(x_coeffs, t, 2);
    double y_1 = EvaluateCubicPolynomial(y_coeffs, t, 1);
    double y_2 = EvaluateCubicPolynomial(y_coeffs, t, 2);
    // cost = curvature * v^2 + time_to_exit
    cost =
        std::max(cost, std::abs(x_1 * y_2 - y_1 * x_2) / std::hypot(x_1, y_1) +
                           time_to_exit);
    t += FLAGS_prediction_trajectory_time_resolution;
  }
  return cost;
}

std::vector<double> JunctionPredictor::GenerateCandidateTimes() {
  std::vector<double> candidate_times;
  double t = 1.0;
  double time_gap = 0.5;
  while (t <= FLAGS_prediction_trajectory_time_length) {
    candidate_times.push_back(t);
    t += time_gap;
  }
  return candidate_times;
}

}  // namespace prediction
}  // namespace apollo
