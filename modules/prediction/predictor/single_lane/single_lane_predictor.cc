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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#include "modules/prediction/predictor/single_lane/single_lane_predictor.h"

#include <memory>
#include <string>
#include <utility>

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::hdmap::LaneInfo;

SingleLanePredictor::SingleLanePredictor() {
  predictor_type_ = ObstacleConf::SINGLE_LANE_PREDICTOR;
}

void SingleLanePredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  obstacle->SetPredictorType(predictor_type_);

  const Feature& feature = obstacle->latest_feature();

  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << "] has no lane graph.";
    return;
  }

  std::string lane_id = "";
  if (feature.lane().has_lane_feature()) {
    lane_id = feature.lane().lane_feature().lane_id();
  }
  int num_lane_sequence = feature.lane().lane_graph().lane_sequence_size();

  for (int i = 0; i < num_lane_sequence; ++i) {
    const LaneSequence& sequence = feature.lane().lane_graph().lane_sequence(i);
    if (sequence.lane_segment_size() <= 0) {
      AERROR << "Empty lane segments.";
      continue;
    }

    ADEBUG << "Obstacle [" << obstacle->id()
           << "] will draw a lane sequence trajectory [" << ToString(sequence)
           << "] with probability [" << sequence.probability() << "].";

    std::vector<TrajectoryPoint> points;
    GenerateTrajectoryPoints(
        *obstacle, sequence, FLAGS_prediction_trajectory_time_length,
        FLAGS_prediction_trajectory_time_resolution, &points);

    if (points.empty()) {
      continue;
    }

    Trajectory trajectory = GenerateTrajectory(points);
    trajectory.set_probability(sequence.probability());
    obstacle->mutable_latest_feature()->add_predicted_trajectory()->CopyFrom(
        trajectory);
  }
}

void SingleLanePredictor::GenerateTrajectoryPoints(
    const Obstacle& obstacle, const LaneSequence& lane_sequence,
    const double time_length, const double time_resolution,
    std::vector<TrajectoryPoint>* points) {
  const Feature& feature = obstacle.latest_feature();
  if (!feature.has_position() || !feature.has_velocity() ||
      !feature.position().has_x() || !feature.position().has_y()) {
    AERROR << "Obstacle [" << obstacle.id()
           << " is missing position or velocity";
    return;
  }

  double probability = lane_sequence.probability();
  double approach_rate = FLAGS_go_approach_rate;
  if (probability < FLAGS_lane_sequence_threshold_cruise) {
    approach_rate = 1.0;
  }

  Eigen::Vector2d position(feature.position().x(), feature.position().y());
  double speed = feature.speed();

  int lane_segment_index = 0;
  std::string lane_id =
      lane_sequence.lane_segment(lane_segment_index).lane_id();
  std::shared_ptr<const LaneInfo> lane_info = PredictionMap::LaneById(lane_id);
  double lane_s = 0.0;
  double lane_l = 0.0;
  if (!PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l)) {
    AERROR << "Failed in getting lane s and lane l";
    return;
  }
  size_t num_of_points = static_cast<size_t>(time_length / time_resolution);
  for (size_t i = 0; i < num_of_points; ++i) {
    double relative_time = static_cast<double>(i) * time_resolution;
    Eigen::Vector2d point;
    double theta = M_PI;
    if (!PredictionMap::SmoothPointFromLane(lane_id, lane_s, lane_l, &point,
                                            &theta)) {
      AERROR << "Unable to get smooth point from lane [" << lane_id
             << "] with s [" << lane_s << "] and l [" << lane_l << "]";
      break;
    }
    TrajectoryPoint trajectory_point;
    PathPoint path_point;
    path_point.set_x(point.x());
    path_point.set_y(point.y());
    path_point.set_z(0.0);
    path_point.set_theta(theta);
    path_point.set_lane_id(lane_id);
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed);
    trajectory_point.set_a(0.0);
    trajectory_point.set_relative_time(relative_time);
    points->emplace_back(std::move(trajectory_point));

    lane_s += speed * time_resolution;

    while (lane_s > PredictionMap::LaneById(lane_id)->total_length() &&
           lane_segment_index + 1 < lane_sequence.lane_segment_size()) {
      lane_segment_index += 1;
      lane_s = lane_s - PredictionMap::LaneById(lane_id)->total_length();
      lane_id = lane_sequence.lane_segment(lane_segment_index).lane_id();
    }

    lane_l *= approach_rate;
  }
}

}  // namespace prediction
}  // namespace apollo
